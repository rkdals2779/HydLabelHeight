#!/usr/bin/env python3.8
from typing import Any
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose


class Control():
    def __init__(self, status="recieve_ID"):
        self.status = status

        self.dwa_status = False
        self.arm_status = False
        self.aruco_status = False

        self.dwa_turn = 0

        self.clicked_point = Pose()

        self.cnt = 0

    def clicked_point_sub(self, msg):
        self.clicked_point = msg

    def dwa_status_sub(self, msg):
        self.dwa_status = msg.data

    def arm_status_sub(self, msg):
        self.arm_status = msg.data

    def aruco_status_sub(self, msg):
        self.aruco_status = msg.data

    def status_pub(self):
        if self.status == 'recieve_ID':
            n = int(input("You Input ID num : "))

            for i in range(n):
                # input id ok?
                pass

            self.status = 'grab'

        if self.status == 'grab':
            arm.pub(pose_arm)
            arm.pub(self.status)

            if self.arm_status == True:
                self.arm_status = False
                self.dwa_turn = 180
                self.status = 'dwa_turn'

        if self.status == 'dwa_turn':
            dwa.pub(dwa_status)

            if self.dwa_turn == 180:
                if self.dwa_status == True:
                    self.dwa_status = False
                    self.status = 'release'

            if self.dwa_turn == 360:
                if self.dwa_status == True:
                    self.dwa_status = False
                    self.status = 'grab'

        if self.status == 'release':
            arm.pub(pose_arm)
            arm.pub(self.status)

            if self.arm_status == True:
                self.arm_status = False
                self.cnt += 1

                if self.cnt >= input_num:
                    self.status = 'cart_on'
                else:
                    self.dwa_turn = 360
                    self.status = 'dwa_turn'

        if self.status == 'cart_on':
            pub(pose_arm)
            pub(self.status)

            if self.arm_status == True:
                self.arm_status = False
                self.status = 'go_destination'

        if self.status == 'go_destination':
            pass


def main():
    control = Control(status)
    while not rospy.is_shutdown():
        control.status_pub()
        print(control.status)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
