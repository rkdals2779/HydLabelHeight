#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose


class L515_Aruco:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
    
    def send_transform(self, pose):
        while not rospy.is_shutdown():
            try:
                self.br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                                      tf.transformations.quaternion_from_euler(-np.pi / 6, np.pi, 0),
                                      rospy.Time.now(),
                                      "Aruco_L515",
                                      "L515"
                                      )
                return 0
            except Exception as e:
                print(e)


# def main():
#     rospy.init_node("Aruco_to_L515")
#     l515 = L515_Aruco()
#     rospy.Subscriber("aruco_to_L515_pose", Pose, l515.send_transform)
#     rospy.spin()


def main():
    rospy.init_node("Aruco_to_L515")
    # l515 = L515_Aruco()
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        try:
            br.sendTransform((0, 0, 0),
                                    tf.transformations.quaternion_from_euler(-np.pi / 6, np.pi, 0),
                                    rospy.Time.now(),
                                    "Aruco_L515",
                                    "L515"
                                    )
        except Exception as e:
            print(e)


if __name__ == "__main__":
    try:
        main()
    except:
        pass
