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
from utils import to_2b, from_2b

p = SerialProxy("/dev/serial/by-path/pci-0000:04:00.3-usb-0:2:1.0-port0", 115200)


def die(n, f):
    p.close()
    exit(0)


signal.signal(signal.SIGINT, die)
signal.signal(signal.SIGTERM, die)

p.subscribe_for_opcode(Opcodes.ODOM, lambda x: print("Got odometry: ", x))
p.subscribe_for_opcode(Opcodes.DIST, lambda x: print("Got distance =", from_2b(*x)))

while True:
    # p.send_packet(Opcodes.GET_DIST)
    p.begin_cmd_packet()
    # for i in range(4):
    p.append_cmd(Opcodes.DRIVE_TARGET, to_2b(400))
    p.append_cmd(Opcodes.TURN_TARGET, to_2b(90))
    p.append_cmd(Opcodes.GET_ODOM)
    p.append_cmd(Opcodes.GET_DIST)
    p.send_cmd_packet()
    time.sleep(1000)
