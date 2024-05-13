#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose


status = False


def call_back(TF):
    global status
    status = TF


def main():
    global status
    rospy.init_node('test_code_just_for_pub_dwa_fin')
    pub = rospy.Publisher('status_pub', String, queue_size=1)
    rospy.Subscriber('dwa_status', Bool, call_back)
    while not status:
        pub.publish('dwa_turn')
    print(123123123123123)


if __name__ == '__main__':
    main()