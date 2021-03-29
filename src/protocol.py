import enum
import threading
import time
from dataclasses import field
from queue import Queue, PriorityQueue

from attr import dataclass
from nti_acs.srv import BridgeSend, BridgeRequestImage
from reedsolo import RSCodec, ReedSolomonError
from serial import Serial

import rospy
import utils

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

Short:
0: Opcode


'''


class Opcodes(enum.IntEnum):
    INTERNAL = 0x00
    C_CAM_REQ = 0x01  # [cam id, None]
    R_CAM_START = 0x05
    R_CAM_DATA = 0x06
    R_CAM_STOP = 0x07
    MIN_USER_OPCODE = 0xA0


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
            Opcodes.C_CAM_REQ: self._send_image_from_cam,
            Opcodes.R_CAM_START: self._start_cam_data,
            Opcodes.R_CAM_STOP: self._stop_cam_data,
            Opcodes.R_CAM_DATA: self._add_cam_data,
        }

        self.subscribers = {}

        self.publishers = {}
        self.user_opcode_handlers = {}

        name = 'serial_proxy_node_' + port_name.replace('/', '')
        rospy.init_node(name)

        self.init_services()

        rospy.loginfo('Started on port ' + port_name)

        # self.stats_pub = rospy.Publisher("/bridge/stats", BridgeStats, queue_size=10, latch=True)

    def init_services(self):
        rospy.Service('/bridge/send', BridgeSend, lambda x: self.send_packet(x.opcode, x.data))
        rospy.Service('/bridge/request_image', BridgeRequestImage, lambda x: self.request_image(x.id))
        # rospy.Service('/bridge/request_image', BridgeRequestImage, lambda x: self.request_image(x.id))

    def init_port(self):
        self._io_thread = threading.Thread(target=self._process_io, daemon=True)
        self._io_thread.start()
        self._worker_thread = threading.Thread(target=self._execute_messages, daemon=True)
        self._worker_thread.start()

    # Camera

    def request_image(self, cid, quality=10):
        self.begin_cmd_packet()
        self.append_cmd(Opcodes.C_CAM_REQ, [cid, quality])
        self.send_cmd_packet()

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
        self.send_packet(Opcodes.R_CAM_START, ext)
        for i in range(len(img) // self.image_chunk_size):
            self.send_packet(Opcodes.R_CAM_DATA, img[i * self.image_chunk_size: (i + 1) * self.image_chunk_size])
        self.send_packet(Opcodes.R_CAM_STOP, [])

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

    def append_cmd(self, opcode, data):
        self._msg_stack.extend(bytes([opcode, *data]))

    def send_cmd_packet(self):
        # self.append_cmd(Opcodes.INTERNAL, [self.END_BYTE, self.END_BYTE])
        self._send(self._msg_stack)
        self._msg_stack.clear()

    def subscribe_for_opcode(self, opcode, cb):
        if opcode < Opcodes.MIN_USER_OPCODE:
            raise ValueError("Not available. Use opcodes starting from Opcodes.MIN_USER_OPCODE")
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
        self._output_packet_queue.put(bytes([*data]))

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
                self._process_input_base()

            self._process_output()

    def _process_output(self):
        if not self._output_packet_queue.empty():
            # w = self._output_packet_queue.get().item
            self.port.write(self._output_packet_queue.get())
            self.port.flushOutput()

    def _process_input_robot(self):
        if self.port.inWaiting() >= 3:
            self._exec_queue.put(self._read(3))

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
        print(opcode)
        if opcode is None:
            if self._resend_req_mode:
                self.send_packet(Opcodes.RESEND, nonce)
            return

        rospy.logdebug("Received: ", nonce, opcode, data, err)
        # TODO send err data

        self._execute_msg(opcode, data)

    def send_packet(self, opcode, data):
        if not self.is_robot:
            if len(data) > 2:
                rospy.logerr("Can't send long message to robot")
                return
            self.begin_cmd_packet()
            self.append_cmd(opcode, data)
            self.send_cmd_packet()
            return
        data = utils.preprocess_types(data)
        raw = self.pack(opcode, data)
        self.sent_packets[self.nonce] = raw
        self._send(raw,
                   priority=(0 if opcode in [Opcodes.R_CAM_DATA,
                                             Opcodes.R_CAM_START,
                                             Opcodes.R_CAM_STOP] else 0))
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
