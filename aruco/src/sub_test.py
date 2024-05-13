#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import tf


def foo(a):
    print(a)


def main():
    rospy.init_node('asldkjfhasdlkjfhgasdlkjfh')
    print(1231123123123)
    rospy.Subscriber('aruco_pose', Pose, foo)
    rospy.spin()


if __name__ == '__main__':
    main()
