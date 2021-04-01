#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from time import sleep

from cam import Camera
from opcodes import Opcodes
from protocol import SerialProxy

p = None

# def die(n, f):
#     p.close()
#     exit(0)
#
#
# signal.signal(signal.SIGINT, die)
# signal.signal(signal.SIGTERM, die)

if len(sys.argv) > 1:
    p = SerialProxy("/dev/pts/4", 115200)  # , topic_postfix='/dev')

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
    # p.append_cmd(Opcodes.WAIT, to_2b(1000))
    p.append_cmd(0xCD, [0, 0])
    # p.append_cmd(Opcodes.WAIT, to_2b(1000))
    # p.append_cmd(0xCC, [0, 0])
    # p.append_cmd(Opcodes.WAIT, to_2b(1000))
    # p.append_cmd(0xCC, [0, 0])
    p.send_cmd_packet()
    # p.send_packet(0xCC, [0xDE, 0xAD])
    sleep(1)
    p.send_packet(Opcodes.INTERNAL, [0xDE, 0xAD])
    # while True:
    #     p.request_image(1, 5)
    #     sleep(10)
    while True:
        pass
else:
    p = SerialProxy("/dev/pts/5", 115200, is_robot=True,
                    camera_controller=Camera())  # , topic_postfix='/dev', camera_controller=Camera())


    def q(x):
        print("SEND")
        return x


    def w(x):
        for i in range(20):
            sleep(0.5)
            print(i)

    def w1(x):
        for i in range(20):
            sleep(0.5)
            print("!", i)


    # p.create_preproc(Opcodes.ANY, (q, w))

    p.subscribe_for_opcode(0xCC, w)
    p.subscribe_for_opcode(0xCD, w1)
    # p.subscribe_for_opcode(0x09, print)
    # p.subscribe_for_opcode(0x47, print)

    while True:
        pass
