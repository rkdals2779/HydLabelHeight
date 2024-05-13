#!/usr/bin/env python3.8
import rospy
import cv2
from cv2 import aruco
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
import tf


aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
cam_mat = np.array([[588.74820, 0, 341.87940],
               [0, 590.14248, 242.68663],
               [0, 0, 1]])
coeffs = np.array([[0.10544], [-0.16979], [-0.01279], [0.00439], [-0.25633]]) # rad1, rad2, tan1, tan2, rad3
# : [K1, K2, tangential1, tangential2, K3].
aruco_size = 0.03
esc_key = 27

class Aruco_id():
    def __init__(self):
        self.id = None

    def get_id(self, id):
        self.id = id.data


def main():
    rospy.init_node("find_aruco_l515")
    pose_pub = rospy.Publisher("aruco_pub", Pose, queue_size=1)
    id_l515 = Aruco_id()
    rospy.Subscriber('aruco_id', Int32, id_l515.get_id)
    listener = tf.TransformListener()

    cap_l515 = cv2.VideoCapture(12) # back

    while not rospy.is_shutdown():
        _, frame = cap_l515.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        parameters = aruco.DetectorParameters_create()
        coners, ids, point = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(coners, aruco_size, cam_mat, coeffs)

        if ids is not None:
            aruco_pose = Pose()
            aruco_pose.position.x = tvecs[0][0][0]
            aruco_pose.position.y = tvecs[0][0][1]
            aruco_pose.position.z = tvecs[0][0][2]
            while not rospy.is_shutdown():
                try:
                    tran, rot = listener.lookupTransform('link_base', 'Aruco_L515', rospy.Time(0))
                    aruco_pose.position.x += tran[0]
                    aruco_pose.position.y += tran[1]
                    aruco_pose.position.z += tran[2]
                    print('aruco_pose')
                    break
                except Exception as e:
                    print(e)
                    pass
            
            for id_ in ids[0]:
                if id_ == id_l515.id:
                    pose_pub.publish(aruco_pose)

    cap_l515.release()


if __name__ == "__main__":
    main()
    