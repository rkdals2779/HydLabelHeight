#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import tf


class Aruco_pub():
    def __init__(self):
        self.aruco_pose = None
        self.aruco_id = None
        self.status = None
        self.aruco_pub = rospy.Publisher('aruco_pose', Pose, queue_size=1)
        self.listener = tf.TransformListener()
    
    def sub_id(self, id):
        self.aruco_id = id
        while not rospy.is_shutdown():
            try:
                aruco_pose = Pose()
                if self.status == 'L515':
                    tran, rot = self.listener.lookupTransform('link_base', 'Aruco_L515', rospy.Time(0))
                elif self.status == 'D455':
                    tran, rot = self.listener.lookupTransform('base_link', 'Aruco_D455', rospy.Time(0))
                aruco_pose.position.x = tran[0]
                aruco_pose.position.y = tran[1]
                aruco_pose.position.z = tran[2]
                self.aruco_pose = aruco_pose
                break
            except:
                pass
            self.pub_aruco_pose_id()
    
    def pub_aruco_pose_id(self):
        if self.aruco_pose is not None and self.aruco_id is not None:
            self.aruco_pub.publish(self.aruco_pose)
            self.aruco_pose = None
            self.status = None
    
    def sub_status(self, status):
        self.status = status



def main():
    rospy.init_node('aruco_pose_pub')
    aruco_pub = Aruco_pub()
    rospy.Subscriber('aruco_pub_id_pub', String, aruco_pub.sub_id)
    rospy.Subscriber('aruco_status', String, aruco_pub.sub_status)
    rospy.spin()


if __name__ == "__main__":
    main()
