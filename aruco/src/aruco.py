#!/usr/bin/env python3
import rospy
import cv2
from cv2 import aruco
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
import tf


aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
cam_mat = np.array([[588.74820, 0, 341.87940],
               [0, 590.14248, 242.68663],
               [0, 0, 1]])
coeffs = np.array([[0.10544], [-0.16979], [-0.01279], [0.00439], [-0.25633]]) # rad1, rad2, tan1, tan2, rad3
# : [K1, K2, tangential1, tangential2, K3].
aruco_size = 0.03
esc_key = 27


class Aruco():
    def __init__(self):
        self.aruco_id = None
        self.pose = Pose()
        self.br = tf.TransformBroadcaster()

    def get_id(self, id):
        self.aruco_id = id.data
    
    def get_pose(self, tvecs):
        self.pose.position.x = tvecs[0][0]
        self.pose.position.y = tvecs[0][1]
        self.pose.position.z = tvecs[0][2]


def main():
    rospy.init_node("find_aruco_l515")
    aruco_cls = Aruco()
    cap_l515 = cv2.VideoCapture(12)

    while not rospy.is_shutdown():
        rospy.Subscriber('aruco_id', Int32, aruco_cls.get_id)
        _, frame = cap_l515.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        parameters = aruco.DetectorParameters_create()
        coners, ids, point = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(coners, aruco_size, cam_mat, coeffs)

        if ids is None:
            continue

        for n, id in enumerate(ids):
            print(ids, aruco_cls.aruco_id)
            if id[0] == aruco_cls.aruco_id:
                print(id, aruco_cls.aruco_id)
                aruco_cls.get_pose(tvecs[n])
                aruco_cls.br.sendTransform(
                    (aruco_cls.pose.position.x, aruco_cls.pose.position.y, aruco_cls.pose.position.z),
                    tf.transformations.quaternion_from_euler(-np.pi / 6, np.pi, 0),
                    rospy.Time.now(),
                    'Aruco_L515',
                    'L515'
                )


if __name__ == '__main__':
    main()