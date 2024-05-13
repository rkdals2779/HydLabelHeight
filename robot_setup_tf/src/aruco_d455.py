#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose


class D455_Aruco:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.pose = Pose()

    def aruco_to_camera(self):
        while not rospy.is_shutdown() and self.pose != Pose():
            try:
                r, p, y = self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z
                self.br.sendTransform((self.pose.position.x, self.pose.position.y, self.pose.position.z),
                                      tf.transformations.quaternion_from_euler(0, np.pi, 0),
                                      rospy.Time.now(),
                                      "Aruco_D455",
                                      "D455"
                                      )
                # forced frame -> just shape not exact value -> error orientation
                # r, p, y = self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z
                # self.br.sendTransform((self.pose.position.x, self.pose.position.y, self.pose.position.z),
                #                       tf.transformations.quaternion_from_euler(r - np.pi / 6, p, y + np.pi / 4),
                #                       rospy.Time.now(),
                #                       "Aruco",
                #                       "L515"
                #                       )
                # fixed frame just position front
                # self.br.sendTransform((self.pose.position.x,self.pose.position.y, self.pose.position.z),
                #                       tf.transformations.quaternion_from_euler(-np.pi / 6, np.pi, 0),
                #                       rospy.Time.now(),
                #                       "Aruco",
                #                       "L515"
                #                       )
                return 0
            except Exception as e:
                print(e)
    
    def save_pose(self, pose):
        self.pose = pose
        self.aruco_to_camera()


def main():
    rospy.init_node("Aruco_to_D455")
    d455 = D455_Aruco()
    rospy.Subscriber("aruco_to_D455_pose", Pose, d455.save_pose)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except:
        pass
