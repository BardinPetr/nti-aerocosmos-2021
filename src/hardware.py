# #!/usr/bin/env python
# # -*- coding: utf-8 -*-

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

from time import sleep, time

from future import standard_library

standard_library.install_aliases()

from builtins import object
from math import sqrt, radians

from nti_acs.msg import CamPos, ManipulatorArrow

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16, Int16
from tf.transformations import euler_from_quaternion


class Hardware(object):
    STEPPER_FULL_TURN = 1000
    cur_stepper_pos = 0

    GRIPPER_OPENED = 80
    GRIPPER_CLOSED = 90

    BASE_SPD = 0.5

    imu = None
    odom = None
    sonar = 0
    light = 0

    COLLIDE_CNT_TO_KILL = 10
    anti_collide_thresh = 0
    _last_collide_event = 0
    _collide_event_cnt = 0
    _anti_collide_cb = None

    def __init__(self, spd_map=1, spd_map_angular=1, anti_collide_cb=None, sensor_ok_cb=None):
        l = False
        self.cam_pos = rospy.Publisher("/ard/cam_pos", CamPos, queue_size=10, latch=l)
        self.vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=l)

        self.man_arrow = rospy.Publisher("/ard/manip_arrow", ManipulatorArrow, queue_size=0, latch=l)
        self.man_grip = rospy.Publisher("/ard/manip_grip", Int16, queue_size=0, latch=l)
        self.man_yaw = rospy.Publisher("/ard/manip_yaw", Int16, queue_size=0, latch=l)

        self.sub_imu = rospy.Subscriber("/imu", Imu, lambda x: self.update_imu(x))
        self.sub_odom = rospy.Subscriber("/odom", Odometry, lambda x: self.update_odom(x), )
        self.sub_dist = rospy.Subscriber("/ard/sonar", UInt16, lambda x: self.update_sonar(x))
        self.sub_light = rospy.Subscriber("/ard/light", UInt16, lambda x: self.update_light(x))

        self.global_spd_coef = spd_map
        self.global_angular_spd_coef = spd_map_angular
        self.anti_collide_cb = anti_collide_cb
        self.sensor_ok_cb = sensor_ok_cb

    def _is_ok(self):
        if self.imu is not None and self.odom is not None and self.sensor_ok_cb is not None:
            self.sensor_ok_cb()

    def update_imu(self, x):
        self.imu = x
        self._is_ok()

    def update_odom(self, x):
        self.odom = x
        self._is_ok()

    def update_light(self, x):
        self.light = x

    def update_sonar(self, x):
        self.sonar = x
        if self.anti_collide_thresh > 0 and x < self.anti_collide_thresh:
            if time() - self._last_collide_event > 5:
                self._collide_event_cnt = 0
            self._collide_event_cnt += 1
            if self._collide_event_cnt > self.COLLIDE_CNT_TO_KILL:
                self.drive(0, 0)
                self._collide_event_cnt = 0
                if self.anti_collide_cb is not None:
                    self.anti_collide_cb()

    def location(self):
        return self.odom.pose.pose.position

    def orientation(self):
        quaternion = self.odom.pose.pose.orientation
        return euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

    @staticmethod
    def dist(a, b):
        return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def drive(self, spd_x, spd_y, spd_coef=None):
        if spd_coef is None:
            spd_coef = self.global_spd_coef
        t = Twist()
        t.linear.x = spd_x * spd_coef
        t.linear.y = spd_y * spd_coef
        self.vel.publish(t)

    def stop(self):
        self.drive(0, 0)

    def turn(self, spd):
        t = Twist()
        t.angular.z = spd * self.global_angular_spd_coef
        self.vel.publish(t)

    def drive_dist(self, dst_cm):
        dst = dst_cm / 100
        if self.odom is None:
            raise SystemError("Odom is not published yet")
        self.drive(self.BASE_SPD, 0)
        start = self.location()
        while self.dist(start, self.location()) < dst:
            pass
        self.stop()

    def turn_angle(self, angle):
        if self.odom is None:
            raise SystemError("Odom is not published yet")
        self.turn(self.BASE_SPD)
        start = self.orientation()[2]
        angle = radians(angle)
        while abs(self.orientation()[2] - start) < angle:
            # print(abs(self.orientation()[2] - start))
            sleep(0.005)
        self.stop()

    def set_anti_collide_thresh(self, thresh):
        self.anti_collide_thresh = thresh

    def circle(self, r, mode):
        pass

    def set_cam_pitch(self, pitch):
        self.cam_pos.publish(pitch, -1)

    def set_cam_yaw(self, yaw):
        cur_stepper_pos = yaw / 360 * self.STEPPER_FULL_TURN
        self.cam_pos.publish(-1, cur_stepper_pos)

    def add_cam_yaw(self, d, yaw):
        self.set_cam_yaw(max(-350, min(350,
                                       self.cur_stepper_pos +
                                       (-1 if d == 0 else 1) * yaw / 360 * self.STEPPER_FULL_TURN)))

    def set_gripper(self, grip=None, angle=None):
        if grip is not None:
            self.set_gripper(angle=(self.GRIPPER_CLOSED if grip else self.GRIPPER_OPENED))
        elif angle is not None:
            self.man_grip.publish(angle)

    def set_manipulator_arrow(self, a0, a1):
        self.man_arrow.publish(a0, a1)

    def set_manipulator_yaw(self, a):
        self.man_yaw.publish(a)

    def place_beacon(self, i):
        pass  # TODO

    def auto_leave(self, thresh):
        while self.light < thresh:
            pass

        sleep(2)
        self.drive(-0.4, 0)
        sleep(2)
        self.drive(0, 0)


if __name__ == "__main__":
    rospy.init_node("main", anonymous=True)
    h = Hardware()

    sleep(1)

    h.set_manipulator_yaw(30)
    h.set_manipulator_arrow(255, 180)
    while True:
        # a, b, c = map(int, inputraw().split())
        # h.set_manipulator_yaw(a)
        h.set_manipulator_arrow(int(input()), int(input()))
        # sleep(1)
        # sleep(1)
