# #!/usr/bin/env python
# # -*- coding: utf-8 -*-

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

from future import standard_library

from cam import Camera
from std_msgs.msg import ColorRGBA

standard_library.install_aliases()
import signal

import rospy
from protocol import SerialProxy, Opcodes
from hardware import Hardware
from utils import from_2b, to_2b, drive_on_robot

rospy.init_node("main", anonymous=True)

ard_status = rospy.Publisher("/ard/status", ColorRGBA, queue_size=10, latch=True)


def led(r, g, b):
    ard_status.publish(ColorRGBA(r, g, b, 0))


led(255, 255, 0)

c = Camera()

p = SerialProxy(
    port_name=rospy.get_param('~modem_port',
                              '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
    is_robot=True,
    baud=rospy.get_param('~modem_baud', 115200),
    started_cb=lambda: led(0, 0, 255),
    camera_controller=c
)

has_odom = False


def has_odom_upd():
    global has_odom
    has_odom = True


d = Hardware(rospy.get_param('~spd_coef', 1),
             rospy.get_param('~ang_spd_coef', 1),
             anti_collide_cb=p.clear_queue,
             sensor_ok_cb=has_odom_upd)

p.subscribe_for_kill(lambda: d.stop())


def die(n, f):
    p.close()
    exit(0)


signal.signal(signal.SIGINT, die)
signal.signal(signal.SIGTERM, die)

# p.create_preproc(Opcodes.ANY, (lambda x: x, lambda x: x))

p.subscribe_for_opcode(Opcodes.GET_DIST,
                       lambda x: p.send_packet(
                           Opcodes.DIST,
                           to_2b(d.sonar)
                       ))
p.subscribe_for_opcode(Opcodes.GET_ODOM,
                       lambda x: p.send_packet(Opcodes.ODOM, d))

p.subscribe_for_opcode(Opcodes.DRIVE,
                       lambda x: d.drive(*drive_on_robot(*x), spd_coef=1))
p.subscribe_for_opcode(Opcodes.DRIVE_TARGET,
                       lambda x: d.drive_dist(from_2b(*x)))
p.subscribe_for_opcode(Opcodes.TURN_TARGET,
                       lambda x: d.turn_angle(from_2b(*x)))
p.subscribe_for_opcode(Opcodes.CIRCLE,
                       lambda x: d.circle(*x))
p.subscribe_for_opcode(Opcodes.AUTO_LEAVE,
                       lambda x: d.auto_leave(500))

p.subscribe_for_opcode(Opcodes.CAM_PITCH,
                       lambda x: d.set_cam_pitch(x[0]))
p.subscribe_for_opcode(Opcodes.CAM_YAW,
                       lambda x: d.set_cam_yaw(from_2b(*x)))
p.subscribe_for_opcode(Opcodes.CAM_YAW_ADD,
                       lambda x: d.add_cam_yaw(x[0], x[1]))

p.subscribe_for_opcode(Opcodes.MANIPULATOR_GRIP,
                       lambda x: d.set_gripper(x[0]))
p.subscribe_for_opcode(Opcodes.MANIPULATOR_ARROW,
                       lambda x: d.set_manipulator_arrow(*x))
p.subscribe_for_opcode(Opcodes.MANIPULATOR_YAW,
                       lambda x: d.set_manipulator_yaw(from_2b(*x)))
p.subscribe_for_opcode(Opcodes.MANIPULATOR_EXEC,
                       lambda x: d.place_beacon(x[0]))
p.subscribe_for_opcode(Opcodes.AUTO_PLACE4,
                       lambda x: d.do_square(from_2b(*x), lambda: p.send_image_from_cam([2, 10])))

rate = rospy.Rate(1)
i = 255
j = 0
while not rospy.is_shutdown():
    i = 255 if i == 0 else 0
    j = 255 if j == 0 else 0
    led(0 if has_odom else 255, i, 0)
    rate.sleep()
