#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose


def main():
    rospy.init_node("L515_to_link_base")
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        try:
            br.sendTransform(
                (0.1, 0, 0.11),
                tf.transformations.quaternion_from_euler(- 2 * np.pi / 3, 0, - np.pi / 2),
                rospy.Time.now(),
                "L515",
                "link_base"
            )
        except Exception as e:
            print(e)
            pass


if __name__ == "__main__":
    try:
        main()
    except:
        pass