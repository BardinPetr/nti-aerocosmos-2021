#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import time

from future import standard_library

standard_library.install_aliases()
import signal

from protocol import SerialProxy, Opcodes
from utils import to_2b, from_2b, drive_to_robot, drive_on_robot

# p = SerialProxy("/dev/serial/by-path/pci-0000:04:00.3-usb-0:2:1.0-port0", 115200)
p = SerialProxy("/dev/ttyUSB1", 19200)


def die(n, f):
    p.close()
    exit(0)


signal.signal(signal.SIGINT, die)
signal.signal(signal.SIGTERM, die)

p.subscribe_for_opcode(Opcodes.ODOM, lambda x: print("Got odometry: ", x))
p.subscribe_for_opcode(Opcodes.DIST, lambda x: print("Got distance =", from_2b(*x)))

def create_pkt(i):
    p.append_cmd(Opcodes.DRIVE_TARGET, to_2b(80))
    p.append_cmd(Opcodes.MANIPULATOR_EXEC, [i, 0])
    p.append_cmd(Opcodes.CAM_REQ, [3, 10])
    p.append_cmd(Opcodes.DRIVE_TARGET, to_2b(20))
    p.append_cmd(Opcodes.TURN_TARGET, to_2b(90))


p.subscribe_for_opcode(0xDD, lambda x: print("OK", x))

while True:
    # p.send_packet(Opcodes.GET_DIST)

    # p.append_cmd(Opcodes.GET_ODOM)
    # p.append_cmd(Opcodes.INTERNAL, [0xDE, 0xAD])
    # p.append_cmd(Opcodes.DRIVE_TARGET, to_2b(100))
    # p.append_cmd(Opcodes.CAM_YAW_ADD, [1, 50])
    # p.append_cmd(Opcodes.MANIPULATOR_GRIP, [1, 0])
    # p.append_cmd(Opcodes.WAIT, to_2b(1))
    # p.append_cmd(Opcodes.MANIPULATOR_GRIP, [0, 0])

    # for i in [1, 2, 0, 1]:

    print(1)
    p.begin_cmd_packet()

    # p.append_cmd(Opcodes.AUTO_LEAVE, to_2b(400))
    # p.append_cmd(0xDD, [0, 0])

    # p.append_cmd(Opcodes.DRIVE, [100, 0])
    # p.append_cmd(Opcodes.WAIT, to_2b(1000))
    # p.append_cmd(Opcodes.DRIVE, [0, 0])
    # p.append_cmd(Opcodes.DRIVE_TARGET, to_2b(200))

    # p.append_cmd(Opcodes.TURN_TARGET, to_2b(270))
    # p.append_cmd(Opcodes.DRIVE_TARGET, to_2b(1000))
    # p.append_cmd(Opcodes.INTERNAL, [0xDE, 0xAD])
    # p.append_cmd(Opcodes.MANIPULATOR_EXEC, [1, 0])
    # create_pkt(0)
    # p.append_cmd(Opcodes.CAM_REQ, [3, 10])
    # p.append_cmd(Opcodes.CAM_REQ, [3, 10])

    p.send_cmd_packet()
    time.sleep(1000)
