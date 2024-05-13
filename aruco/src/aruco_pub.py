#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import tf


def main():
    rospy.init_node('aruco_pose_pub')
    aruco_pub = rospy.Publisher('aruco_pose', Pose, queue_size=1)
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            aruco_pose = Pose()
            tran, rot = listener.lookupTransform('link_base', 'Aruco_L515', rospy.Time(0))
            aruco_pose.position.x = tran[0]
            aruco_pose.position.y = tran[1]
            aruco_pose.position.z = tran[2]

            aruco_pub.publish(aruco_pose)
        except Exception as e:
            print(e)
    rospy.spin()


# def main():
#     rospy.init_node('aruco_pose_pub')
#     aruco_pub = rospy.Publisher('aruco_pose', Pose, queue_size=1)
#     listener = tf.TransformListener()
#     listener.waitForTransform('link_base', 'Aruco_L515', rospy.Time(), rospy.Duration(1.0))
#     while not rospy.is_shutdown():
#         try:
#             aruco_pose = Pose()
#             now = rospy.Time.now()
#             listener.waitForTransform('link_base', 'Aruco_L515', now, rospy.Duration(1.0))
#             tran, rot = listener.lookupTransform('link_base', 'Aruco_L515', now)
#             aruco_pose.position.x = tran[0]
#             aruco_pose.position.y = tran[1]
#             aruco_pose.position.z = tran[2]

#             aruco_pub.publish(aruco_pose)
#         except Exception as e:
#             print(e)
#     rospy.spin()


if __name__ == "__main__":
    main()
