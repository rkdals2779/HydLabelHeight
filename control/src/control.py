#!/usr/bin/env python3.8
from typing import Any
import rospy
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose

class Control():
    def __init__(self) -> None:
        self.pose = Pose()
        self.TF = False

        self.status_pub = rospy.Publisher('status_pub', String, queue_size=1)
        self.arm_point_pub = rospy.Publisher('arm_point', Pose, queue_size=1)
        self.aruco_id_pub = rospy.Publisher('aruco_id', Int32, queue_size=1)

    def pose_sub(self, pose):
        self.pose = pose
        print(self.pose)
    
    def arm_sub(self, TF):
        self.TF = TF
        print(self.TF)


def main():
    rospy.init_node('pub_aaaa')
    control = Control()
    rospy.Subscriber('aruco_pub', Pose, control.pose_sub)
    rospy.Subscriber('arm_status', Bool, control.arm_sub)
    while not control.TF:
        control.aruco_id_pub.publish(6)
        if control.pose != Pose():
            print(control.pose)
            print(control.TF)
            control.status_pub.publish('cart_on')
            control.arm_point_pub.publish(control.pose)


if __name__ == '__main__':
    main()
