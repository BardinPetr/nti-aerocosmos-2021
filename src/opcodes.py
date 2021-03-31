from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from future import standard_library
standard_library.install_aliases()
import enum


class Opcodes(enum.IntEnum):
    INTERNAL = 0x00
    # [0x5A, 0x9D] - packet_start
    # [0x0E, 0x4D] - packet_end
    # [0xDE, 0x1E] - clear rover cmd queue

    # 2b - 2 byte integer

    # ROVER -> PC
    CAM_START = 0x01                    # [number 0-255 of added padding end symbols to make round packets] // packet sent before CAM_DATA
    CAM_DATA = 0x02                     # [*img_data] // image data
    CAM_STOP = 0x03                     # [] // packet sent after all CAM_DATA packets

    DIST = 0x04                         # [2b cm] // sonar distance
    ODOM = 0x05                         # [python pickled [x, y, yaw]] // odometry pos + estimated yaw

    # PC -> ROVER
    WAIT = 0x07                         # [seconds 2B]

    DRIVE = 0xA0                        # [speed x, speed y] set speed (/cmd_vel), mapped from [0-255] to [-spd_max;spd_max]
    DRIVE_TARGET = 0xA1                 # [dist cm 2B] // drive forward for distance in cm
    TURN_TARGET = 0xA2                  # [angle deg 2B] // turn around for angle in deg
    TURN_TILL = 0xA3                    # [mode, val] ! reserved for future releases
    CIRCLE = 0xA4                       # [radius, mode] ! reserved for future releases

    CAM_PITCH = 0xB0                    # [angle deg 2B] [0-180] // set cam pos
    CAM_YAW = 0xB1                      # [angle deg 2B] [0-360]
    CAM_REQ = 0xB2                      # [cam dev id, JPEG quality] // request image from robots camera in JPEG

    MANIPULATOR_GRIP = 0xB4             # [0-1, None] // set manipulator gripper status
    MANIPULATOR_ARROW = 0xB5            # [arrow motor pos 1, arrow motor pos 2] set manipulator arrows' servos to angles in [0-180] or 252 to ignore and save current
    MANIPULATOR_YAW = 0xB6              # [angle deg 2B] // turn manipulator in deg
    MANIPULATOR_EXEC = 0xB7             # [] automated take beacon and place it

    GET_DIST = 0xC0                     # [] -> DIST // request sonar packet
    GET_ODOM = 0xC1                     # [] -> ODOM // request odometry packet
