#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose


def main():
    rospy.init_node("velodyne_to_base_link")
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        try:
            br.sendTransform(
                (-0.3, 0, -0.25),
                (0, 0, 0, 1),
                rospy.Time.now(),
                "base_link",
                "velodyne"
            )
        except Exception as e:
            print(e)
            pass


if __name__ == '__main__':
    main()