#!/usr/bin/env python3.8
from typing import Any
import rospy
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose
import tf


arm_status = False


def asdfasdf(msg):
    global arm_status
    arm_status = msg


def main():
    global arm_status
    rospy.init_node('control_scout_2023_testtesttest')
    rospy.Subscriber('arm_status', Bool, asdfasdf)
    pose = Pose()
    listener = tf.TransformListener()
    sleep = rospy.Rate(1)
    while pose == Pose():
        rospy.Publisher('aruco_id', Int32, queue_size=1).publish(6)
        try:
            trans, rot = listener.lookupTransform('link_base', 'Aruco_L515', rospy.Time(0))
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            print(pose)
        except:
            pass
    
    sleep.sleep()

    while not arm_status:
        rospy.Publisher('arm_point', Pose, queue_size=1).publish(pose)
        rospy.Publisher('status_pub', String, queue_size=1).publish('cart_on')


if __name__ == '__main__':
    main()