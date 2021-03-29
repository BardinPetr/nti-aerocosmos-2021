import enum
import threading
from dataclasses import field
from queue import PriorityQueue

from attr import dataclass
from nti_acs.srv import BridgeSend, BridgeSubscribeTopic, BridgeRequestImage, BridgeStats
from reedsolo import RSCodec, ReedSolomonError
from serial import Serial

import rospy
import utils

'''
Message description:

Base:
0-1: START_BYTES
2: SIZE (3+n+m)
3: NONCE
4: OPCODE
[5;5+n]: DATA len=n
[6+n; 6+n+m]: REED-SOLOMON CODE len=m
7+n+m: END_BYTE
'''


class Opcodes(enum.IntEnum):
    INTERNAL = 0x00
    SUB_TOPIC = 0x01
    USUB_TOPIC = 0x02
    CAM_REQ = 0x04
    CAM_START = 0x05
    CAM_DATA = 0x06
    CAM_STOP = 0x07
    RESEND = 0x08
    ROS_TOPIC = 0x0A


@dataclass(order=True)
class PrioritizedMessage:
    priority: int
    item: bytes = field(compare=False)


class SerialProxy:
    SERIAL_TIMEOUT = 5
    SERIAL_TIMEOUT_FAST = 1

    MSG_MAXSIZE = 255

    START_BYTES = bytes([0x31, 0x64])
    END_BYTE = 0x99

    _strict_drop_mode = True
    _resend_req_mode = False
    _pass_first_header = False

    _output_packet_queue = PriorityQueue()

    nonce = 0

    _img_buffer = bytearray()
    _img_ext = None

    _run = True

    def __init__(self,
                 port_name, baud, rsc_n=12,
                 is_robot=False,
                 strict_drop_mode=False,
                 resend_req_enabled=False,
                 topic_postfix='',
                 image_chunk_size=235,
                 camera_controller=None
                 ):
        self.is_robot = is_robot
        self.sent_packets = {}
        self.rsc_n = rsc_n
        self.rsc = RSCodec(rsc_n, nsize=self.MSG_MAXSIZE)

        self.port = Serial(port_name, baud, timeout=self.SERIAL_TIMEOUT, write_timeout=self.SERIAL_TIMEOUT)
        self.init_port()

        self._strict_drop_mode = strict_drop_mode
        self._resend_req_mode = resend_req_enabled
        self._camera_controller = camera_controller

        self.image_chunk_size = image_chunk_size
        self.topic_postfix = topic_postfix

        self.handlers = {
            Opcodes.SUB_TOPIC: self._subscribe_request,
            Opcodes.USUB_TOPIC: self._unsubscribe_request,
            Opcodes.CAM_REQ: self._send_image_from_cam,
            Opcodes.CAM_START: self._start_cam_data,
            Opcodes.CAM_STOP: self._stop_cam_data,
            Opcodes.CAM_DATA: self._add_cam_data,
            Opcodes.ROS_TOPIC: self._topic_update,
            Opcodes.RESEND: self._resend_msg
        }

        self.subscribers = {}

        self.publishers = {}
        self.user_opcode_handlers = {}

        name = 'serial_proxy_node_' + port_name.replace('/', '')
        rospy.init_node(name)

        self.init_services()

        rospy.loginfo('Started on port ' + port_name)

        self.stats_pub = rospy.Publisher("/bridge/stats", BridgeStats, queue_size=10, latch=True)

    def init_services(self):
        rospy.Service('/bridge/subscribe_topic', BridgeSubscribeTopic, lambda x: self.subscribe(x.name))
        rospy.Service('/bridge/unsubscribe_topic', BridgeSubscribeTopic, lambda x: self.unsubscribe(x.name))
        rospy.Service('/bridge/send', BridgeSend, lambda x: self.send_packet(x.opcode, x.data))
        rospy.Service('/bridge/request_image', BridgeRequestImage, lambda x: self.request_image(x.id))

    def init_port(self):
        self._write_thread = threading.Thread(target=self._process_output, daemon=True)
        self._write_thread.start()
        self._read_thread = threading.Thread(target=self.spin, daemon=True)
        self._read_thread.start()

    # Subscribe procedure

    # STEP 0 DEV0
    def subscribe(self, topic):
        rospy.loginfo('Got user request for receiving topic updates ' + topic)
        self.send_packet(Opcodes.SUB_TOPIC, topic.encode('utf-8'))
        enc = utils.encode_topic(topic)
        msg_type = utils.msg_object_by_topic(topic)
        self.publishers[enc] = rospy.Publisher(topic + self.topic_postfix, msg_type, queue_size=0, latch=True)
        return 0

    # STEP 1 DEV1
    def _subscribe_request(self, data):
        topic = data.decode('utf-8')
        rospy.loginfo('Got subscribe request from other node for ' + topic)
        if topic not in self.subscribers:
            self.subscribers[topic] = rospy.Subscriber(
                topic,
                utils.msg_object_by_topic(topic),
                lambda x: self._subscriber(utils.encode_topic(data), x)
            )

    # STEP 2 DEV1 
    def _subscriber(self, topic, data):
        rospy.loginfo('Forwarding message to serial: ' + str(data))
        self.send_packet(Opcodes.ROS_TOPIC, [topic, data])

    # STEP 3 DEV0
    def _topic_update(self, raw):
        t_enc = bytes(raw[:2])
        rospy.loginfo('Forwarding message to ROS: ' + str(raw))
        if t_enc not in self.publishers:
            return
        pub = self.publishers[t_enc]
        new_msg = pub.data_class()
        new_msg.deserialize(raw[2:])
        pub.publish(new_msg)

    # Subscribe procedure

    # STEP 0 DEV0
    def unsubscribe(self, topic):
        rospy.loginfo('Got user request to UNsubscribe topic ' + topic)
        self.send_packet(Opcodes.USUB_TOPIC, topic.encode('utf-8'))
        enc = utils.encode_topic(topic)
        if enc in self.publishers:
            self.publishers[enc].unregister()
            del self.publishers[enc]
        return 0

    # STEP 1 DEV1
    def _unsubscribe_request(self, data):
        topic = data.decode('utf-8')
        rospy.loginfo('Got UNsubscribe request from other node for ' + topic)
        if topic in self.subscribers:
            self.subscribers[topic].unregister()
            del self.subscribers[topic]

    # Non-ROS opcodes (when using this class as part of another module)

    def subscribe_for_opcode(self, opcode, cb):
        self.user_opcode_handlers[opcode] = cb

    def aux_opcode_handler(self, opcode, data):
        if opcode in self.user_opcode_handlers:
            self.user_opcode_handlers[opcode](data)

    # Camera

    def request_image(self, cid, quality=10):
        self.send_packet(Opcodes.CAM_REQ, [cid, quality])

    def _send_image_from_cam(self, data):
        if self._camera_controller is None:
            return
        rospy.loginfo(f"Sending image from cam {data[0]} with quality {data[1]}")
        img = self._camera_controller.get(*data)
        self.send_image(img)

    def send_image(self, img):
        ext = self.image_chunk_size - (len(img) % self.image_chunk_size)
        img = bytearray(img)
        img.extend([0x00] * ext)
        self.send_packet(Opcodes.CAM_START, ext)
        for i in range(len(img) // self.image_chunk_size):
            self.send_packet(Opcodes.CAM_DATA, img[i * self.image_chunk_size: (i + 1) * self.image_chunk_size])
        self.send_packet(Opcodes.CAM_STOP, [])

    def _add_cam_data(self, data):
        print("ADD", len(data))
        if self._img_ext is None:
            return
        self._img_buffer.extend(data)

    def _start_cam_data(self, data):
        self._img_ext = data[0]
        self._img_buffer.clear()

    def _stop_cam_data(self, data):
        res = self._img_buffer[:-self._img_ext]
        print("END", self._img_ext, len(res))
        self._img_ext = None
        open('res.jpg', 'wb').write(res)

    # Serial port methods

    def _resend_msg(self, nonce):
        self._send(self.sent_packets[nonce])

    def close(self):
        if self._camera_controller is not None:
            self._camera_controller.close()
        self.port.close()
        self._run = False
        self._write_thread.join()
        self._read_thread.join()

    def _send(self, data, priority=0):
        self._output_packet_queue.put(PrioritizedMessage(priority, data))

    def _read(self, cnt=1):
        return self.port.read(cnt)

    def _read_byte(self):
        return int(self.port.read()[0])

    def _process_output(self):
        while not rospy.is_shutdown() and self._run:
            if not self._output_packet_queue.empty():
                self.port.write(self._output_packet_queue.get().item)
                self.port.flushOutput()

    def spin(self):
        self._pass_first_header = False
        while not rospy.is_shutdown() and self._run:
            self._process_input()

    def _process_input(self):
        if self.port.inWaiting() < 2:
            return

        err_cnt = 0
        if self._read_byte() != SerialProxy.START_BYTES[0] and (not self._pass_first_header):
            return
        else:
            err_cnt += 1

        self._pass_first_header = False
        nxt = self._read_byte()
        if nxt == SerialProxy.START_BYTES[0]:
            self._pass_first_header = True
            return
        if nxt != SerialProxy.START_BYTES[1]:
            if self._strict_drop_mode:
                return
            self.port.timeout = self.SERIAL_TIMEOUT_FAST
            err_cnt += 1

        size = self._read_byte()
        if err_cnt > 0:
            self.port.setTimeout = self.SERIAL_TIMEOUT

        raw = self._read(size)  # TODO add processing with messages that have lower len

        if raw[-1] != SerialProxy.END_BYTE and self._strict_drop_mode:
            return

        nonce, opcode, data, err = self.unpack(raw[:-1])

        if opcode is None:
            if self._resend_req_mode:
                self.send_packet(Opcodes.RESEND, nonce)
            return

        rospy.logdebug("Received: ", nonce, opcode, data, err)
        # TODO about err

        try:
            opcode = Opcodes(opcode)
            if opcode in self.handlers:
                self.handlers[opcode](data)
                return
        except ValueError:
            pass
        self.aux_opcode_handler(opcode, data)

    def send_packet(self, opcode, data):
        data = utils.preprocess_types(data)
        raw = self.pack(opcode, data)
        self.sent_packets[self.nonce] = raw
        self._send(raw, priority=(1 if opcode == Opcodes.CAM_DATA else 0))
        return 0

    # Serialization/deserialization

    def unpack(self, raw):
        try:
            decoded, _, err_pos = self.rsc.decode(raw)
            return decoded[0], decoded[1], decoded[2:], len(err_pos)  # nonce, opcode, data, error_count
        except ReedSolomonError:
            return None, None, None, None

    def pack(self, opcode, data):
        self.nonce = (self.nonce + 1) % 255
        size = 3 + len(data) + self.rsc_n

        opcode = opcode if type(opcode) is int else opcode.value
        data = [self.nonce, opcode, *data]
        if opcode < 0 or opcode > 255:
            raise ValueError('Invalid opcode')
        if size >= SerialProxy.MSG_MAXSIZE:
            raise DataLengthError(size - SerialProxy.MSG_MAXSIZE)

        data = self.rsc.encode(data)
        return bytes([*SerialProxy.START_BYTES, size, *data, SerialProxy.END_BYTE])


class DataLengthError(ValueError):
    pass


class InvalidHeaderError(Exception):
    pass


class UnrecoverablePacketError(Exception):
    pass
