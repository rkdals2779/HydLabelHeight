#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
import rospy
import tf
import numpy as np

def main():
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
	# mani
        br.sendTransform((-0.25, 0.0, 0.08),
                          tf.transformations.quaternion_from_euler(0, 0, np.pi),
                         rospy.Time.now(),
                         "link_base",
                         "base_link")
	# velodyne
        br.sendTransform((0.265, 0.0, 0.29),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "velodyne",
                         "base_link")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
