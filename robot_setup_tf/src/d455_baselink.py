#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose


def main():
    rospy.init_node("D455_to_base_link")
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        try:
            br.sendTransform(
                (0.46, 0, 0),
                tf.transformations.quaternion_from_euler(-np.pi / 2, 0, - np.pi / 2),
                rospy.Time.now(),
                "D455",
                "base_link"
            )
        except Exception as e:
            print(e)
            pass


if __name__ == "__main__":
    try:
        main()
    except:
        pass