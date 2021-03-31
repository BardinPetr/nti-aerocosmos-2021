from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

from future import standard_library

standard_library.install_aliases()
import signal

import rospy
from protocol import SerialProxy, Opcodes
from hardware import Hardware
from utils import from_2b, to_2b

rospy.init_node("main", anonymous=True)

p = SerialProxy(
    rospy.get_param('~modem_port',
                    '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
    rospy.get_param('~modem_baud', 115200),
    rospy.get_param('~spd_coef', 0.05),
    rospy.get_param('~ang_spd_coef', 0.05)
)


def die(n, f):
    p.close()
    exit(0)


signal.signal(signal.SIGINT, die)
signal.signal(signal.SIGTERM, die)

d = Hardware()

p.subscribe_for_opcode(Opcodes.GET_DIST,
                       lambda x: p.send_packet(
                           Opcodes.DIST,
                           to_2b(d.sonar)
                       ))
p.subscribe_for_opcode(Opcodes.GET_ODOM,
                       lambda x: p.send_packet(Opcodes.ODOM, d))

p.subscribe_for_opcode(Opcodes.DRIVE,
                       lambda x: d.drive(*x, spd_coef=1))
p.subscribe_for_opcode(Opcodes.DRIVE_TARGET,
                       lambda x: d.drive_dist(from_2b(*x)))
p.subscribe_for_opcode(Opcodes.TURN_TARGET,
                       lambda x: d.turn_angle(from_2b(*x)))
p.subscribe_for_opcode(Opcodes.CIRCLE,
                       lambda x: d.circle(*x))

p.subscribe_for_opcode(Opcodes.CAM_PITCH,
                       lambda x: d.set_cam(from_2b(*x), -1))
p.subscribe_for_opcode(Opcodes.CAM_YAW,
                       lambda x: d.set_cam(-1, from_2b(*x)))

p.subscribe_for_opcode(Opcodes.MANIPULATOR_GRIP,
                       lambda x: d.set_manipulator(-1, -1, -1, x[0]))
p.subscribe_for_opcode(Opcodes.MANIPULATOR_ARROW,
                       lambda x: d.set_manipulator(*(list(x) + [-1, -1])))
p.subscribe_for_opcode(Opcodes.MANIPULATOR_YAW,
                       lambda x: d.set_manipulator(-1, -1, from_2b(*x), -1))
p.subscribe_for_opcode(Opcodes.MANIPULATOR_EXEC,
                       lambda x: d.place_beacon(x[0]))

while True:
    pass
