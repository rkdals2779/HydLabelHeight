#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose, PoseArray


class camera_br:
    def __init__(self):
        self.L515_arucos = PoseArray()
        self.L515_detect = False
        self.D455_aruco01 = Pose()
        self.D455_detect = False
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber('L515_arucos_pose', PoseArray, self.L515_arucos_pose)
        # rospy.Subscriber('D455_arucos_pose', PoseArray, self.D455_arucos_pose)
        print("start")

    def brs(self):
        while not rospy.is_shutdown():
            # L515
            self.br.sendTransform((0.1015, 0.0, 0.085),
                                  tf.transformations.quaternion_from_euler(-np.pi * 2 / 3, 0, -np.pi / 2),
                                  rospy.Time.now(),
                                  "L515",
                                  "link_base")
            if self.L515_detect:
                for i in range(len(self.L515_arucos.poses)):
                    if i > 1:
                        minus = -1
                    else:
                        minus = 1
                    try:
                        self.br.sendTransform(
                            (self.L515_arucos.poses[i].position.x, self.L515_arucos.poses[i].position.y,
                             self.L515_arucos.poses[i].position.z),
                            (tf.transformations.quaternion_from_euler(self.L515_arucos.poses[i].orientation.x,
                                                                      minus * self.L515_arucos.poses[i].orientation.z,
                                                                      -self.L515_arucos.poses[i].orientation.y,
                                                                      'rxyz')),
                            rospy.Time.now(),
                            f"L515_aruco{int(self.L515_arucos.poses[i].orientation.w)}",
                            "L515")
                    except:
                        print("L515 error occurred ### i : ", i)
                try:
                    self.br.sendTransform(
                        (0, 0, -0.03),
                        (tf.transformations.quaternion_from_euler(0, 0, 0)),
                        rospy.Time.now(),
                        "cart_pose",
                        "L515_aruco2")
                except:
                    print("2 in none")

            # D455
            self.br.sendTransform((0.38, 0.0, 0.125),
                                  tf.transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2),
                                  rospy.Time.now(),
                                  "D455",
                                  "base_link")

            if self.D455_detect:
                for i in range(len(self.D455_arucos.poses)):
                    try:
                        self.br.sendTransform(
                            (self.D455_arucos.poses[i].position.x, self.D455_arucos.poses[i].position.y,
                             self.D455_arucos.poses[i].position.z),
                            (tf.transformations.quaternion_from_euler(self.D455_arucos.poses[i].orientation.x,
                                                                      self.D455_arucos.poses[i].orientation.z,
                                                                      -self.D455_arucos.poses[i].orientation.y,
                                                                      'rxyz')),
                            rospy.Time.now(),
                            f"D455_aruco{int(self.D455_arucos.poses[i].orientation.w)}",
                            "D455")
                    except:
                        print("D455 error occurred ### i : ", i)

    def L515_arucos_pose(self, a):
        self.L515_arucos = a
        self.L515_detect = True

    def D455_arucos_pose(self, a):
        self.D455_arucos = a
        self.D455_detect = True


def main():
    rospy.init_node("Cameras_broadcaster")
    camera = camera_br()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        camera.brs()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
