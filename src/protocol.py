# #!/usr/bin/env python
# # -*- coding: utf-8 -*-

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import sys
from threading import Thread

from future import standard_library

standard_library.install_aliases()

from builtins import bytes
from builtins import int
from builtins import object
from builtins import open
from builtins import range
from builtins import str

import time
from queue import Queue
import os
from nti_acs.srv import BridgeSend, BridgeRequestImage, BridgeSubscribeTopic
from reedsolo import RSCodec, ReedSolomonError
from serial import Serial

import rospy
import utils

from opcodes import Opcodes

'''
Message description (full):
Base:
0-1: START_BYTES
2: SIZE (3+n+m)
3: NONCE
4: OPCODE
[5;5+n]: DATA len=n
[6+n; 6+n+m]: REED-SOLOMON CODE len=m
7+n+m: END_BYTE
'''


# @dataclass(order=True)
# class PrioritizedMessage(object):
#     priority: int
#     item: bytes = field(compare=False)


def _sleep(raw):
    time.sleep(utils.from_2b(raw[0], raw[1]) / 1000)


class SerialProxy(object):
    SERIAL_TIMEOUT = 5
    SERIAL_TIMEOUT_FAST = 1

    MSG_MAXSIZE = 255

    START_BYTES = bytes([0x31, 0x64])
    END_BYTE = 0x99

    _strict_drop_mode = True
    _resend_req_mode = False
    _pass_first_header = False

    # _output_packet_queue = PriorityQueue()
    _output_packet_queue = Queue()

    _exec_queue = Queue()

    nonce = 0

    _img_buffer = bytearray()
    _img_ext = None

    _run = True

    _msg_stack = bytearray()

    def __init__(self,
                 port_name, baud, rsc_n=12,
                 is_robot=False,
                 strict_drop_mode=False,
                 resend_req_enabled=False,
                 topic_postfix='',
                 image_chunk_size=235,
                 camera_controller=None,
                 reed_solomon=False,
                 started_cb=None,
                 post_kill_cb=None
                 ):
        self.post_kill_cb = post_kill_cb
        self.reed_solomon = reed_solomon
        self.is_robot = is_robot
        self.sent_packets = {}
        self.rsc_n = rsc_n
        self.rsc = RSCodec(rsc_n, nsize=self.MSG_MAXSIZE)
        self.sending_img = 0
        self.port = Serial(port_name, baud, timeout=self.SERIAL_TIMEOUT, write_timeout=self.SERIAL_TIMEOUT)
        self.init_port()

        self._strict_drop_mode = strict_drop_mode
        self._resend_req_mode = resend_req_enabled
        self._camera_controller = camera_controller

        self.image_chunk_size = image_chunk_size
        self.topic_postfix = topic_postfix

        self.handlers = {
            Opcodes.CAM_REQ: self.send_image_from_cam,
            Opcodes.CAM_START: self._start_cam_data,
            Opcodes.CAM_STOP: self._stop_cam_data,
            Opcodes.CAM_DATA: self._add_cam_data,
            Opcodes.WAIT: _sleep
        }

        self.preprocessors = {
            Opcodes.ODOM: (utils.preprocess_odom, utils.postprocess_odom)
        }

        self.subscribers = {}

        self.publishers = {}
        self.user_opcode_handlers = {}

        name = 'serial_proxy_node'
        # rospy.init_node(name, anonymous=True)

        self.init_services()

        rospy.loginfo('Started on port ' + port_name)

        if started_cb is not None:
            started_cb()

    def init_services(self):
        rospy.Service('/bridge/subscribe_topic', BridgeSubscribeTopic, lambda x: self.subscribe(x.name))
        rospy.Service('/bridge/unsubscribe_topic', BridgeSubscribeTopic, lambda x: self.unsubscribe(x.name))
        rospy.Service('/bridge/send', BridgeSend, lambda x: self.send_packet(x.opcode, x.data))
        rospy.Service('/bridge/request_image', BridgeRequestImage, lambda x: self.request_image(x.id))
        # rospy.Service('/bridge/request_image', BridgeRequestImage, lambda x: self.request_image(x.id))

    def init_port(self):
        self._io_thread = Thread(target=lambda: self._process_io())
        self._io_thread.daemon = True
        self._io_thread.start()
        self._worker_thread = Thread(target=lambda: self._execute_messages())
        self._worker_thread.daemon = True
        self._worker_thread.start()

    def subscribe_for_kill(self, x):
        self.post_kill_cb = x

    def clear_queue(self):
        self._exec_queue.queue.clear()

    def _internal_cmd_handler(self, a, b):
        if a == 0xDE:
            # sys.exit(0)
            self.clear_queue()
            print("Clearing packet queue")
            rospy.loginfo("Clearing packet queue")
            # try:
            #     self._worker_thread.kill()
            # except:
            #     pass

            try:
                self._worker_thread = Thread(target=lambda: self._execute_messages())
                self._worker_thread.daemon = True
                self._worker_thread.start()
            except:
                pass

            if self.post_kill_cb is not None:
                self.post_kill_cb()
            self.init_port()
        elif a == 0xDD:
            print("Restarting self")
            os.execv(sys.argv[0], sys.argv)

    # Camera

    def request_image(self, cid, quality=10):
        self.begin_cmd_packet()
        self.append_cmd(Opcodes.CAM_REQ, [cid, quality])
        self.send_cmd_packet()

    def send_image_from_cam(self, data, retry=10):
        if self._camera_controller is None:
            return
        rospy.loginfo("Sending image from cam")
        img = self._camera_controller.get(data[0], data[1])
        if img is not None:
            open(time.ctime() + '.jpg', 'wb').write(img)
            self.send_image(img)
        elif retry > 0:
            self.send_image_from_cam(data, retry - 1)

    def send_image(self, img):
        ext = self.image_chunk_size - (len(img) % self.image_chunk_size)
        img = bytearray(img)
        img.extend([0x00] * ext)
        print("sending image of", len(img), "in packets", len(img) // self.image_chunk_size)
        self.sending_img = 2 + len(img) // self.image_chunk_size
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
        print("Got img", self._img_ext, len(res))
        self._img_ext = None
        open(time.ctime() + '.jpg', 'wb').write(res)

    # 3b packet handling

    def begin_cmd_packet(self):
        self._msg_stack.clear()
        # self.append_cmd(Opcodes.INTERNAL, self.START_BYTES)

    def append_cmd(self, opcode, data=[0, 0]):
        if Opcodes.ANY in self.preprocessors:
            data = self.preprocessors[Opcodes.ANY][0](data)
        self._msg_stack.extend(bytes([opcode] + list(data)))

    def send_cmd_packet(self):
        # self.append_cmd(Opcodes.INTERNAL, [self.END_BYTE, self.END_BYTE])
        self._send(self._msg_stack)
        self._msg_stack.clear()

    def subscribe_for_opcode(self, opcode, cb):
        if opcode in self.handlers:
            raise ValueError("Not available.")
        self.user_opcode_handlers[opcode] = cb

    def aux_opcode_handler(self, opcode, data):
        if opcode in self.user_opcode_handlers:
            self.user_opcode_handlers[opcode](data)

    # Serial port methods

    def _resend_msg(self, nonce):
        self._send(self.sent_packets[nonce])

    def close(self):
        if self._camera_controller is not None:
            self._camera_controller.close()
        self.port.close()
        self._run = False
        self._io_thread.join()
        self._worker_thread.join()

    def _send(self, data, priority=0):
        # self._output_packet_queue.put(PrioritizedMessage(priority, bytes([*data])))
        self._output_packet_queue.put(bytes(data))

    def _read(self, cnt=1):
        return self.port.read(cnt)

    def _read_byte(self):
        return int(self.port.read()[0])

    def _execute_messages(self):
        while not rospy.is_shutdown() and self._run:
            if not self._exec_queue.empty():
                data = self._exec_queue.get()
                self._execute_msg(data[0], data[1:])

    def _execute_msg(self, opcode, data):
        try:
            opcode = Opcodes(opcode)
            if opcode in self.preprocessors:
                data = self.preprocessors[opcode][1](data)
            if opcode in self.handlers:
                self.handlers[opcode](data)
                return
        except ValueError:
            pass
        self.aux_opcode_handler(opcode, data)

    # IO threaded processing

    def _process_io(self):
        self._pass_first_header = False
        while not rospy.is_shutdown() and self._run:
            if self.is_robot:
                self._process_input_robot()
            else:
                try:
                    self._process_input_base()
                except Exception as ex:
                    print(ex)

            self._process_output()

    def _process_output(self):
        if not self._output_packet_queue.empty():
            # w = self._output_packet_queue.get().item
            self.port.write(self._output_packet_queue.get())
            self.port.flushOutput()
            if self.sending_img > 0:
                time.sleep(0.2)
                self.sending_img -= 1
            else:
                time.sleep(0.05)

    def _process_input_robot(self):
        if self.port.inWaiting() >= 3:
            d = bytes(self._read(3))
            if Opcodes.ANY in self.preprocessors:
                d = self.preprocessors[Opcodes.ANY][1](d)
            if d[0] == Opcodes.INTERNAL:
                return self._internal_cmd_handler(d[1], d[2])
            self._exec_queue.put(d)

    def _process_input_base(self):
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
        print(Opcodes(opcode))
        if opcode is None:
            if self._resend_req_mode:
                self.send_packet(Opcodes.RESEND, nonce)
            return

        rospy.logdebug("Received: ", nonce, opcode, data, err)
        # TODO send err data

        self._execute_msg(opcode, data)

    def create_preproc(self, opcode, cb):
        self.preprocessors[opcode] = cb

    def send_packet(self, opcode, data=[0, 0]):
        data = utils.preprocess_types(data)
        if opcode in self.preprocessors:
            data = self.preprocessors[opcode][0](data)

        if not self.is_robot:
            if len(data) > 2:
                rospy.logerr("Can't send long message to robot")
                return
            self.begin_cmd_packet()
            self.append_cmd(opcode, data)
            self.send_cmd_packet()
            return

        if Opcodes.ANY in self.preprocessors:
            data = self.preprocessors[Opcodes.ANY][0](data)

        raw = self.pack(opcode, data)
        self.sent_packets[self.nonce] = raw
        self._send(raw,
                   priority=(0 if opcode in [Opcodes.CAM_DATA,
                                             Opcodes.CAM_START,
                                             Opcodes.CAM_STOP] else 0))
        return 0

    # Serialization/deserialization

    def unpack(self, raw):
        if not self.reed_solomon:
            return raw[0], raw[1], raw[2:], 0
        try:
            decoded, _, err_pos = self.rsc.decode(raw)
            return decoded[0], decoded[1], decoded[2:], len(err_pos)  # nonce, opcode, data, error_count
        except ReedSolomonError:
            return None, None, None, None

    def pack(self, opcode, data):
        self.nonce = (self.nonce + 1) % 255
        size = 3 + len(data) + (self.rsc_n if self.reed_solomon else 0)

        opcode = opcode if type(opcode) is int else opcode.value
        data = [self.nonce, opcode] + list(data)
        if opcode < 0 or opcode > 255:
            raise ValueError('Invalid opcode')
        if size >= SerialProxy.MSG_MAXSIZE:
            raise DataLengthError(size - SerialProxy.MSG_MAXSIZE)
        if self.reed_solomon:
            data = self.rsc.encode(data)
        return SerialProxy.START_BYTES + bytes([size]) + bytearray(data) + bytes([SerialProxy.END_BYTE])

    # ROS TOPIC PROXY

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


class DataLengthError(ValueError):
    pass


class InvalidHeaderError(Exception):
    pass


class UnrecoverablePacketError(Exception):
    pass
