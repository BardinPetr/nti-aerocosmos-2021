import enum


class Opcodes(enum.IntEnum):
    INTERNAL = 0x00
    # [0x5A, 0x9D] - packet_start
    # [0x0E, 0x4D] - packet_end
    # [0xDE, 0x1E] - clear rover cmd queue

    # ROVER -> PC
    CAM_START = 0x01                    # [number 0-255 of added padding end symbols to make round packets]
    CAM_DATA = 0x02                     # [*img_data]
    CAM_STOP = 0x03                     # []

    DIST = 0x04                         # [2b cm]
    ODOM = 0x05                         # [python pickled [x, y, yaw]]

    # PC -> ROVER
    WAIT = 0x07                         # [seconds 2B]

    DRIVE = 0xA0                        # [speed left, speed right] spd mapped from [0-255] to [-spd_max;spd_max]
    DRIVE_TARGET = 0xA1                 # [dist cm 2B]
    TURN_TARGET = 0xA2                  # [angle deg 2B]
    TURN_TILL = 0xA3                    # [mode, val] !reserved for future releases
    CIRCLE = 0xA4                       # [radius, mode] !reserved for future releases

    CAM_PITCH = 0xB0                    # [angle deg 2B] [0-180]
    CAM_YAW = 0xB1                      # [angle deg 2B] [0-360]
    CAM_REQ = 0xB2                      # [cam dev id, JPEG quality]

    MANIPULATOR_GRIP = 0xB4             # [0-1, None]
    MANIPULATOR_ARROW = 0xB5            # [arrow motor pos 1, arrow motor pos 2] [0-180] 252 to ignore and save current
    MANIPULATOR_YAW = 0xB6              # [angle deg 2B]
    MANIPULATOR_EXEC = 0xB7             # [] automated take beacon and place it

    GET_DIST = 0xC0                     # [] -> DIST
    GET_ODOM = 0xC1                     # [] -> ODOM
