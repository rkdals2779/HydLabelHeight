#!/usr/bin/env python3.8
from typing import Any
import rospy
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose
import tf


class Control():
    def __init__(self, status='grab'):
        self.status = status
        self.dwa_status = False
        self.arm_status = False
        self.aruco_ids = []
        self.aruco_pose = Pose()
        self.dwa_turn = 0
        self.clicked_point = Pose()
        self.total_cnt = 0
        self.cnt = 0
        self.pubbed = False
        self.listener = tf.TransformListener()

        self.pre_pose = Pose()

        self.status_pub = rospy.Publisher('status_pub', String, queue_size=1)
        self.arm_point_pub = rospy.Publisher('arm_point', Pose, queue_size=1)
        self.dwa_point_pub = rospy.Publisher('dwa_point', Pose, queue_size=1)
        self.aruco_id_pub = rospy.Publisher('aruco_id', Int32, queue_size=1)
    
    def get_aruco_ids(self, aruco_ids):
        self.aruco_ids = aruco_ids
        self.total_cnt = len(aruco_ids)

    def clicked_point_sub(self, pose):
        self.clicked_point = pose

    def dwa_status_sub(self, msg):
        self.dwa_status = msg.data

    def arm_status_sub(self, msg):
        self.arm_status = msg.data

    def pub_control(self):
        self.aruco_id_pub.publish(self.aruco_ids[self.cnt])
        if self.aruco_pose == Pose() and self.status != 'grab':
            try:
                tvecs, rvecs = self.listener.lookupTransform('link_base', 'Aruco_L515', rospy.Time(0))
                self.aruco_pose.position.x = tvecs[0]
                self.aruco_pose.position.y = tvecs[1]
                self.aruco_pose.position.z = tvecs[2]
                self.aruco_id_pub.publish(-1)
            except Exception as e:
                print('pub fail')
                return 0
        
        print(self.status, self.arm_status, self.dwa_status)

        if self.status == 'grab':
            self.arm_point_pub.publish(self.aruco_pose)
            self.aruco_pose = Pose()
            if self.arm_status == True:
                self.arm_status = False
                self.dwa_turn = 180
                self.status = 'dwa_turn'

        if self.status == 'dwa_turn':
            self.aruco_ids[self.cnt] = -1
            if self.dwa_status == True:
                self.dwa_status = False
                if self.dwa_turn == 180:
                    self.status = 'release'
                if self.dwa_turn == 360:
                    self.status = 'grab'

        if self.status == 'release':
            if self.arm_status == True:
                self.arm_status = False
                self.cnt += 1
                if self.cnt >= self.total_cnt:
                    self.status = 'cart_on'
                    self.cnt = 0
                    self.aruco_ids = [6]
                    return 0
                else:
                    self.dwa_turn = 360
                    self.status = 'dwa_turn'

        if self.status == 'cart_on':
            if self.aruco_pose != Pose():
                self.arm_point_pub.publish(self.aruco_pose)
            if self.arm_status == True:
                self.arm_status = False
                self.status = 'go_goal'

        if self.status == 'go_goal':
            self.dwa_point_pub.publish(self.clicked_point)
            if self.dwa_status == True:
                print('모든 동작을 완료했습니다!!!')
                exit()
        self.status_pub.publish(self.status)


def main():
    rospy.init_node('control_scout_2023')
    control = Control()
    total_cnt = int(input('물건의 갯수를 입력해주세요 : '))
    aruco_ids = []
    for i in range(total_cnt):
        id = int(input('번호를 입력해주세요 : '))
        aruco_ids.append(id)
    control.get_aruco_ids(aruco_ids=aruco_ids)
    rospy.Subscriber('clicked_point', Pose, control.clicked_point_sub)
    rospy.Subscriber('dwa_status', Bool, control.dwa_status_sub)
    rospy.Subscriber('arm_status', Bool, control.arm_status_sub)
    while not rospy.is_shutdown():
        control.pub_control()
    # while not rospy.is_shutdown():
    #     pass



if __name__ == '__main__':
    main()