#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import cv2
import glob
import numpy as np
import math
from cv2 import aruco
import tf

import argparse
import sys
import os
import math

import rospy
import tf
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseArray, Pose

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(7, 5, 0.038, 0.03, aruco_dict)
cam = cv2.VideoCapture(12)  # usb cam
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

L515_arucos_pose = rospy.Publisher('L515_arucos_pose', PoseArray, queue_size=1)
L515_arucos_num = rospy.Publisher('L515_arucos_num', Int32, queue_size=1)




def calibrate_charuco():
    allCorners = []
    allIds = []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
    images = glob.glob(f'/home/scout/catkin_ws/src/2022TransportationRobot/aruco/src/L515_aruco_imgs/charuco*.png')
    imsize = None
    for im in images:
        # print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners) > 0:
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize=(3, 3),
                                 zeroZone=(-1, -1),
                                 criteria=criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        imsize = gray.shape

    return allCorners, allIds, imsize


def calibrate_camera(allCorners, allIds, imsize):
    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[669.37154771, 0., 505.64274607],
                                 [0., 670.45322216, 285.59950482],
                                 [0., 0., 1.]])

    distCoeffsInit = np.array([[0.12525728],
                               [-0.36984266],
                               [-0.00255186],
                               [0.00676359],
                               [0.22344382]])
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    # flags = (cv2.CALIB_RATIONAL_MODEL)
    # (ret, camera_matrix, distortion_coefficients0,
    #  rotation_vectors, translation_vectors,
    #  stdDeviationsIntrinsics, stdDeviationsExtrinsics,
    #  perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
    #     charucoCorners=allCorners,
    #     charucoIds=allIds,
    #     board=board,
    #     imageSize=imsize,
    #     cameraMatrix=cameraMatrixInit,
    #     distCoeffs=distCoeffsInit,
    #     flags=flags,
    #     criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))
    # print("ret")
    # print(ret)
    # print("camera matrix")
    # print(camera_matrix)
    # print("rotation_vectors")
    # print(rotation_vectors)
    # print("translation_vectors")
    # print(translation_vectors)
    # print("distortion")
    # print(distortion_coefficients0)
    ret = 0.2959894673677669
    camera_matrix = np.array([[669.37154771, 0., 505.64274607],
                              [0., 670.45322216, 285.59950482],
                              [0., 0., 1.]])
    distortion_coefficients0 = np.array([[3.56606412e+00],
                                         [4.36527424e+01],
                                         [-1.59822497e-03],
                                         [7.95922165e-03],
                                         [1.56659502e+02],
                                         [3.61727355e+00],
                                         [4.15532455e+01],
                                         [1.60376532e+02],
                                         [0.00000000e+00],
                                         [0.00000000e+00],
                                         [0.00000000e+00],
                                         [0.00000000e+00],
                                         [0.00000000e+00],
                                         [0.00000000e+00]])
    rotation_vectors = [np.array([[-2.00806663],
                       [ 0.22681015],
                       [ 0.13197846]]), np.array([[-2.99407125],
                       [ 0.08406701],
                       [ 0.22441618]]), np.array([[-0.54156932],
                       [-2.99275749],
                       [-0.04765407]]), np.array([[-2.83119455],
                       [ 0.97448332],
                       [ 0.51594461]]), np.array([[ 2.64887899],
                       [ 0.03728457],
                       [-0.75519074]]), np.array([[ 0.1079947 ],
                       [-2.56796651],
                       [ 1.16351937]]), np.array([[-2.18749569],
                       [ 0.07633526],
                       [-0.17581735]]), np.array([[-0.14263269],
                       [-2.71639055],
                       [ 1.14297394]]), np.array([[ 3.02595887],
                       [-0.05558997],
                       [-0.61054493]]), np.array([[ 0.07363874],
                       [-2.58121368],
                       [ 1.19984129]]), np.array([[ 1.57196898],
                       [ 1.65932976],
                       [-1.11218496]]), np.array([[-2.80092356],
                       [ 0.1998918 ],
                       [ 0.09081918]]), np.array([[-0.11341527],
                       [-2.52495537],
                       [ 1.49079628]]), np.array([[ 0.32808692],
                       [-3.01623886],
                       [-0.3808377 ]]), np.array([[-3.04153664],
                       [ 0.14935944],
                       [ 0.23045123]]), np.array([[-2.23226043],
                       [ 1.98509557],
                       [ 0.20378566]]), np.array([[ 2.30954536],
                       [-0.00340552],
                       [-0.21403343]]), np.array([[ 2.49637184],
                       [-1.77055927],
                       [-0.39606149]]), np.array([[ 2.56021728],
                       [-1.50971587],
                       [-0.35749753]]), np.array([[ 2.15396049],
                       [-0.55050726],
                       [-0.31443481]]), np.array([[-3.04678047],
                       [ 0.04196825],
                       [ 0.10409944]]), np.array([[-2.71983268],
                       [ 0.68840447],
                       [ 0.10266489]]), np.array([[ 0.55194379],
                       [-2.99959312],
                       [-0.30643803]]), np.array([[ 1.59327396],
                       [ 1.69872084],
                       [-1.15670167]]), np.array([[-0.02974067],
                       [-2.50559051],
                       [ 1.41797374]]), np.array([[-2.79021852],
                       [ 1.06999925],
                       [ 0.55640387]]), np.array([[ 2.8508484 ],
                       [-0.04000915],
                       [-0.23339345]]), np.array([[-2.97996314],
                       [ 0.10948414],
                       [ 0.19991858]]), np.array([[ 2.40097751],
                       [ 0.22581198],
                       [-0.53794549]]), np.array([[ 0.11855416],
                       [-2.53788481],
                       [ 1.10653877]]), np.array([[ 1.13700613],
                       [-2.86202419],
                       [-0.63174073]]), np.array([[ 2.47688162],
                       [ 1.35424045],
                       [-0.99184442]]), np.array([[ 2.62879777],
                       [-1.41075296],
                       [-0.61118702]]), np.array([[ 1.67063   ],
                       [ 1.85444878],
                       [-0.96180274]]), np.array([[-0.09675795],
                       [-2.53212018],
                       [ 1.44462803]]), np.array([[ 1.29402236],
                       [ 2.24746298],
                       [-1.45034749]]), np.array([[-3.06247962],
                       [-0.03926573],
                       [ 0.25075658]]), np.array([[-0.23091026],
                       [-2.74780259],
                       [ 1.17881151]]), np.array([[-2.22345002],
                       [ 0.25330597],
                       [ 0.18413476]]), np.array([[-2.63437026],
                       [ 0.58923135],
                       [ 0.13448066]]), np.array([[ 2.5121079 ],
                       [ 0.22794758],
                       [-0.46974429]]), np.array([[ 0.06415729],
                       [-2.47346409],
                       [ 1.28018612]]), np.array([[ 2.81919174],
                       [-0.01152751],
                       [-0.69099497]]), np.array([[-0.05557764],
                       [-2.51088899],
                       [ 1.42177854]]), np.array([[-2.40443249],
                       [ 1.83653091],
                       [ 0.37047213]]), np.array([[ 2.25941015],
                       [-0.85084634],
                       [-0.48888209]]), np.array([[-2.70690534],
                       [ 0.40690185],
                       [ 0.12382192]]), np.array([[ 2.5835305 ],
                       [-0.02880395],
                       [-0.43164566]]), np.array([[-2.94928849],
                       [-0.05337128],
                       [ 0.10692128]]), np.array([[-0.54153842],
                       [ 2.970854  ],
                       [ 0.71037358]]), np.array([[ 2.22457472],
                       [ 0.01793184],
                       [-0.25747235]]), np.array([[-0.15499918],
                       [-2.85617864],
                       [ 0.00491357]]), np.array([[-0.10285007],
                       [-2.53071761],
                       [ 1.48222291]]), np.array([[-2.84117054],
                       [ 0.18724171],
                       [ 0.1201625 ]]), np.array([[ 1.79231808],
                       [ 1.9253325 ],
                       [-0.78142659]]), np.array([[-1.86165742],
                       [ 0.01405042],
                       [-0.22575665]]), np.array([[-0.01927043],
                       [-2.4847985 ],
                       [ 1.3798235 ]]), np.array([[-3.08679437],
                       [ 0.05383678],
                       [ 0.09372769]]), np.array([[-0.85715806],
                       [-2.92933348],
                       [-0.17179198]]), np.array([[ 1.09337679],
                       [ 2.48905   ],
                       [-1.45009383]]), np.array([[-2.9216967 ],
                       [ 0.07091563],
                       [ 0.2951432 ]]), np.array([[-3.02149852],
                       [ 0.13786302],
                       [ 0.25354594]]), np.array([[-2.00993328],
                       [ 0.02674416],
                       [-0.15205983]]), np.array([[-3.04703593],
                       [ 0.17942445],
                       [ 0.30676499]]), np.array([[-2.01220813],
                       [ 2.05803088],
                       [ 0.19076659]]), np.array([[-2.71939618],
                       [-1.29957185],
                       [ 0.84766519]]), np.array([[-2.86102359],
                       [ 0.87676071],
                       [ 0.3668323 ]]), np.array([[ 1.3969002 ],
                       [-2.71068966],
                       [-0.56805447]]), np.array([[-2.99226491],
                       [ 0.06933864],
                       [ 0.18379833]]), np.array([[ 2.43481121],
                       [-0.89923445],
                       [-0.43311411]]), np.array([[-2.62824423],
                       [ 0.04895112],
                       [-0.13866874]]), np.array([[-2.46996943],
                       [ 0.22449716],
                       [ 0.24375132]]), np.array([[-2.85218312],
                       [ 0.09027267],
                       [ 0.35905048]]), np.array([[-2.38588224],
                       [ 0.05407325],
                       [-0.13807579]]), np.array([[-1.77517601],
                       [ 2.14086404],
                       [ 0.24474119]]), np.array([[-0.77923404],
                       [ 2.95046816],
                       [ 0.70168803]]), np.array([[-3.07819742],
                       [ 0.20126653],
                       [ 0.33332548]]), np.array([[-3.11696408],
                       [ 0.30431449],
                       [ 0.13764075]])]
    translation_vectors = [np.array([[-0.0829567],
       [-0.05541868],
       [ 0.72251519]]), np.array([[-0.09132771],
       [ 0.06007683],
       [ 0.52039336]]), np.array([[ 0.13942077],
       [-0.14889054],
       [ 0.4425255 ]]), np.array([[0.02503192],
       [0.14476457],
       [0.52339314]]), np.array([[-0.03237123],
       [ 0.0253526 ],
       [ 0.66342908]]), np.array([[ 0.16003367],
       [-0.12293768],
       [ 0.58091689]]), np.array([[-0.17250211],
       [ 0.03947472],
       [ 0.50218291]]), np.array([[ 0.10493253],
       [-0.09508914],
       [ 0.50607951]]), np.array([[-0.0179956],
       [ 0.0181847],
       [ 0.6985468]]), np.array([[ 0.14222173],
       [-0.0923415 ],
       [ 0.54014997]]), np.array([[-0.05520787],
       [-0.14245102],
       [ 0.74059387]]), np.array([[-0.09902156],
       [ 0.07944135],
       [ 0.4811482 ]]), np.array([[ 0.10920087],
       [-0.06243553],
       [ 0.50781486]]), np.array([[ 0.25562105],
       [-0.07022843],
       [ 0.42986532]]), np.array([[-0.09428597],
       [ 0.1047634 ],
       [ 0.45968874]]), np.array([[0.16847387],
       [0.12619605],
       [0.49546003]]), np.array([[-0.18574967],
       [ 0.05022034],
       [ 0.30122104]]), np.array([[0.17432112],
       [0.13240404],
       [0.49343641]]), np.array([[0.10273052],
       [0.14674706],
       [0.46723271]]), np.array([[-0.06731628],
       [ 0.08323037],
       [ 0.37734613]]), np.array([[-0.15789742],
       [ 0.08355943],
       [ 0.36469767]]), np.array([[-0.01887479],
       [ 0.13933389],
       [ 0.48011141]]), np.array([[ 0.25106961],
       [-0.05823123],
       [ 0.4408817 ]]), np.array([[-0.04670387],
       [-0.13645392],
       [ 0.75745464]]), np.array([[ 0.1579006 ],
       [-0.11029419],
       [ 0.66092623]]), np.array([[0.04050542],
       [0.16009827],
       [0.51513815]]), np.array([[-0.19640929],
       [ 0.08449872],
       [ 0.3351823 ]]), np.array([[-0.20542247],
       [ 0.10216013],
       [ 0.43518768]]), np.array([[-0.07501743],
       [-0.02381928],
       [ 0.56831051]]), np.array([[ 0.18317141],
       [-0.16103564],
       [ 0.65081285]]), np.array([[0.22398877],
       [0.00730076],
       [0.49054967]]), np.array([[-0.09525499],
       [-0.07507341],
       [ 0.58061361]]), np.array([[0.09816706],
       [0.13270792],
       [0.55153434]]), np.array([[-0.04529129],
       [-0.17721918],
       [ 0.72521796]]), np.array([[ 0.13245044],
       [-0.08122149],
       [ 0.6002285 ]]), np.array([[ 0.02310772],
       [-0.12002811],
       [ 0.61625925]]), np.array([[-0.27784165],
       [ 0.07934799],
       [ 0.41908979]]), np.array([[ 0.09862253],
       [-0.07227899],
       [ 0.47044235]]), np.array([[-0.08156707],
       [-0.01933849],
       [ 0.69673354]]), np.array([[-0.03170546],
       [ 0.11824812],
       [ 0.51795094]]), np.array([[-0.06697798],
       [-0.02793156],
       [ 0.57420314]]), np.array([[ 0.21225069],
       [-0.13885137],
       [ 0.75104914]]), np.array([[-0.02464193],
       [ 0.02495254],
       [ 0.67962232]]), np.array([[ 0.14942478],
       [-0.09815026],
       [ 0.63517951]]), np.array([[0.12989084],
       [0.1351083 ],
       [0.51266458]]), np.array([[-0.02932862],
       [ 0.11315729],
       [ 0.4261055 ]]), np.array([[-0.06356865],
       [ 0.09953189],
       [ 0.50494613]]), np.array([[-0.19747767],
       [ 0.0805438 ],
       [ 0.34962006]]), np.array([[-0.12375002],
       [ 0.05778719],
       [ 0.45872105]]), np.array([[ 0.2635708 ],
       [-0.04488343],
       [ 0.46467282]]), np.array([[-0.18263304],
       [ 0.03631212],
       [ 0.30864771]]), np.array([[ 0.1223427 ],
       [-0.15284163],
       [ 0.43866626]]), np.array([[ 0.11728199],
       [-0.06427323],
       [ 0.56428491]]), np.array([[-0.11126566],
       [ 0.08056095],
       [ 0.48103651]]), np.array([[-0.05580368],
       [-0.20102682],
       [ 0.69101466]]), np.array([[-0.18626391],
       [-0.01317464],
       [ 0.51347783]]), np.array([[ 0.17568661],
       [-0.11873431],
       [ 0.69453917]]), np.array([[-0.15655089],
       [ 0.10255116],
       [ 0.324371  ]]), np.array([[ 0.13176419],
       [-0.15107076],
       [ 0.4296442 ]]), np.array([[ 0.04718898],
       [-0.15319687],
       [ 0.62738723]]), np.array([[-0.04768238],
       [ 0.02880172],
       [ 0.63420753]]), np.array([[-0.15123828],
       [ 0.10296235],
       [ 0.45603785]]), np.array([[-0.18249963],
       [-0.00117338],
       [ 0.52232997]]), np.array([[-0.01970796],
       [ 0.09663208],
       [ 0.50756467]]), np.array([[0.19167582],
       [0.11029761],
       [0.50464645]]), np.array([[-0.10684102],
       [-0.05195544],
       [ 0.56041961]]), np.array([[-0.00049064],
       [ 0.13047538],
       [ 0.4708573 ]]), np.array([[0.19951923],
       [0.03212634],
       [0.49474051]]), np.array([[-0.12780795],
       [ 0.07627079],
       [ 0.45550183]]), np.array([[-0.01646172],
       [ 0.12034848],
       [ 0.44192275]]), np.array([[-0.17630222],
       [ 0.07272588],
       [ 0.42795248]]), np.array([[-0.08497556],
       [ 0.0085519 ],
       [ 0.6607766 ]]), np.array([[-0.02102134],
       [ 0.0117204 ],
       [ 0.71687588]]), np.array([[-0.17645016],
       [ 0.05063639],
       [ 0.47982888]]), np.array([[0.23489053],
       [0.08846333],
       [0.52472113]]), np.array([[ 0.25294682],
       [-0.0296032 ],
       [ 0.48345155]]), np.array([[0.02289022],
       [0.09413374],
       [0.51536557]]), np.array([[0.02898536],
       [0.10624291],
       [0.48856158]])]

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors


def detect_marker(matrix, distortion):
    parameters = cv2.aruco.DetectorParameters_create()

    if cam.isOpened():
        while True:
            _, frame = cam.read()
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            coners, ids, point = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=parameters)
            arucos = PoseArray()
            if np.all(ids is not None):
                rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(coners, 0.02, matrix, distortion)
                frame = cv2.aruco.drawAxis(frame, matrix, distortion, rvecs[0], tvecs[0], 0.02)
                tvecs_msg = tvecs.tolist()
                rvecs_msg = rvecs.tolist()
                tvecs_msg_x = tvecs_msg[0][0][0]
                tvecs_msg_y = tvecs_msg[0][0][1]
                tvecs_msg_z = tvecs_msg[0][0][2]
                rvecs_msg_x = rvecs_msg[0][0][0]
                rvecs_msg_y = rvecs_msg[0][0][1]
                rvecs_msg_z = rvecs_msg[0][0][2]

                for i in range(len(ids[:])):
                    aruco01 = Pose()
                    aruco01.position.x = tvecs_msg[i][0][0]
                    aruco01.position.y = tvecs_msg[i][0][1]
                    aruco01.position.z = tvecs_msg[i][0][2]
                    aruco01.orientation.x = rvecs_msg[i][0][0]
                    aruco01.orientation.y = rvecs_msg[i][0][1]
                    aruco01.orientation.z = rvecs_msg[i][0][2]
                    # 하나의 변수로 id까지 보내기 위해 임시로 w에 id를 대입
                    aruco01.orientation.w = ids[i][0]
                    arucos.poses.append(aruco01)
            # print(len(arucos.poses))
            L515_arucos_pose.publish(arucos)
            L515_arucos_num.publish(len(arucos.poses))
            frame = cv2.aruco.drawDetectedMarkers(frame, coners, ids)
            cv2.imshow('video', frame)

            k = cv2.waitKey(1) & 0xff  # press 'esc'to kill
            if k == 27:
                break
        cam.release()


def main():
    rospy.init_node("L515_aruco_xyz_rospy")
    allCorners, allIds, imsize = calibrate_charuco()
    ret, mtx, dist, rvec, tvec = calibrate_camera(allCorners, allIds, imsize)
    detect_marker(mtx, dist)


if __name__ == "__main__":
    main()
