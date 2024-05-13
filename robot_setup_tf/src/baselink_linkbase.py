#!/usr/bin/env python3
import rospy
import tf
import numpy as np

def main():
    rospy.init_node('base_link_to_link_base')
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        br.sendTransform((-0.25, 0.0, 0.08),
                          tf.transformations.quaternion_from_euler(0, 0, np.pi),
                         rospy.Time.now(),
                         "link_base",
                         "base_link")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
