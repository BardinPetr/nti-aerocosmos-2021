#!/usr/bin/env python
# -*- coding: utf-8 -*-

import signal
import sys
from time import sleep

# p.subscribe_for_opcode(0x44, lambda x: p.send_packet(0x99, 0x23))
from opcodes import Opcodes
from protocol import SerialProxy
from utils import to_2b

p = None


def die(n, f):
    p.close()
    exit(0)


signal.signal(signal.SIGINT, die)
signal.signal(signal.SIGTERM, die)

if len(sys.argv) > 1:
    p = SerialProxy("/dev/pts/21", 115200)  # , topic_postfix='/dev')

    #
    # # p.append_cmd(0x09, [0x34, 0x76])
    # # p.append_cmd(0x47, [0x43, 0x69])
    # def q(x):
    #     print("SEND")
    #     return x
    #
    #
    # def w(x):
    #     print("RECV")
    #     return x


    # p.create_preproc(Opcodes.ANY, (q, w))
    p.begin_cmd_packet()
    p.append_cmd(0xCC, [0, 0])
    p.append_cmd(Opcodes.WAIT, to_2b(1000))
    p.append_cmd(0xCC, [0, 0])
    p.append_cmd(Opcodes.WAIT, to_2b(1000))
    p.append_cmd(0xCC, [0, 0])
    p.append_cmd(Opcodes.WAIT, to_2b(1000))
    p.append_cmd(0xCC, [0, 0])
    p.send_cmd_packet()
    sleep(2)
    p.send_packet(Opcodes.INTERNAL, [0xDE, 0xAD])
    while True:
        pass
else:
    p = SerialProxy("/dev/pts/22", 115200, is_robot=True)  # , topic_postfix='/dev', camera_controller=Camera())

    def q(x):
        print("SEND")
        return x


    def w(x):
        print("RECV")
        return x


    p.create_preproc(Opcodes.ANY, (q, w))

    p.subscribe_for_opcode(0xCC, lambda x: print("XXX"))
    # p.subscribe_for_opcode(0x09, print)
    # p.subscribe_for_opcode(0x47, print)

    while True:
        pass
