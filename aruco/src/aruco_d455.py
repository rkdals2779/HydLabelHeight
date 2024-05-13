#!/usr/bin/env python3.8
import rospy
import cv2
from cv2 import aruco
import numpy as np
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Pose
import tf


aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
cam_mat = np.array([[385.94378, 0, 315.59803],
               [0, 382.80951, 237.46623],
               [0, 0, 1]])
coeffs = np.array([[-0.06287], [0.08312], [-0.00252], [0.00199], [-0.04118]]) # rad1, rad2, tan1, tan2, rad3
aruco_size = 0.03
esc_key = 27

class ID():
    def __init__(self) -> None:
        rospy.Subscriber('aruco_id', String, self.get_id)
        self.id = 1

    def get_id(self, id=2):
        self.id = id


def main():
    rospy.init_node("find_aruco_d455")
    pose_pub = rospy.Publisher("aruco_to_D455_pose", Pose, queue_size=1)
    num_pub = rospy.Publisher("aruco_to_D455_id", Int32, queue_size=1)
    pose_to_aruco_pub = rospy.Publisher("d455_pub_pose_pub", Pose, queue_size=1)
    id_d455 = ID()

    cap_d455 = cv2.VideoCapture(4) # front

    poses = np.zeros((3, 100))
    i = 0

    while True:
        i += 1
        _, frame = cap_d455.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        parameters = aruco.DetectorParameters_create()
        coners, ids, point = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # _, rvec, tvec = cv2.solvePnP(objPoint, imgPoint, matA, distCoeff)
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(coners, aruco_size, cam_mat, coeffs)

        if ids is not None:
            frame = cv2.aruco.drawAxis(frame, cam_mat, coeffs, rvecs[0], tvecs[0], aruco_size)

            aruco_pose = Pose()
            aruco_pose.position.x = tvecs[0][0][0]
            aruco_pose.position.y = tvecs[0][0][1]
            aruco_pose.position.z = tvecs[0][0][2]
            # orientation 값 제대로 안들어감
            # aruco_pose.orientation.x = rvecs[0, 0, 0] # roll
            # aruco_pose.orientation.y = rvecs[0, 0, 1] # pitch
            # aruco_pose.orientation.z = rvecs[0, 0, 2] # yaw
            
            for id_ in ids[0]:
                if True : # id_ == id_d455.id:
                    pose_pub.publish(aruco_pose)
                    pose_to_aruco_pub.publish(aruco_pose)
                    print(aruco_pose)

        k = cv2.waitKey(1) & 0xff
        cv2.imshow("frame", frame)
        if k == esc_key:
            cv2.destroyAllWindows()
            cap_d455.release()


if __name__ == "__main__":
    main()
    