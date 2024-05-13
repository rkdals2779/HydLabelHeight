#!/usr/bin/env python3.8

import numpy as np
import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose, Pose2D, PointStamped, Point
from open3d import open3d as o3d
# from math import radians

status = "wait"
dwa_status = False
status_pub = rospy.Publisher('dwa_status', Bool, queue_size=1)

class DWA:
    def __init__(self, publisher):
        rospy.Subscriber('current_pose', Pose, self.slam_callback)
        # rospy.Subscriber('which_print', String, self.print_callback)
        self.publisher = publisher
        self.scout_pose = Pose2D()
        self.click_goal = Point()
        self.home_pos = [[0,0]]
        self.cnt = 0

        self.goal = list()
        self.check_point = 0

    def lds_callback(self, lds):
        turtle_vel = Twist()
        # mps = [0.3, 0.6, 0.9, 1.33, 1.34, 1.35, 1.36, 1.4, 1.45, 1.5]
        radps = [0, 0.1, 0.3, 0.5, 0.7, 0.9, -0.1, -0.3, -0.5, -0.7]
        mps = [0.3, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.4, 0.45, 0.5]
        # radps = [0, 0.1, 0.3, 0.5, 0.7, 0.9, -0.1, -0.3, -0.5, -0.7]
        # mps = [0.2, 0.22, 0.24, 0.26, 0.28, 0.3, 0.32, 0.35, 0.36, 0.38]
        # radps = [0, 0.1, 0.2, 0.3, 0.4, 0.5, -0.1, -0.2, -0.3, -0.4]
        # radps = [0, 0.1, 0.3, 0.5, 0.7, 0.9, -0.1, -0.3, -0.5, -0.7]

        pc_data = pc2.read_points(lds, field_names=("x", "y", "z"), skip_nans=True)
        pc_list = list(pc_data)
        pc_o3d = o3d.geometry.PointCloud()
        pc_o3d.points = o3d.utility.Vector3dVector(pc_list)
        data = pc_o3d

        data_array = self.preprocessing(data)
        local_pos = self.make_combination(mps, radps)
        pos_candidates = self.create_pos_candidates(local_pos)

        self.goal = [self.click_goal.x, self.click_goal.y]

        best_score, back_check = self.evaluate_scores(pos_candidates, self.goal, data_array, local_pos)
        turtle_vel.linear.x, turtle_vel.angular.z = mps[best_score[0]], radps[best_score[1]]
        if back_check:
            turtle_vel.linear.x = mps[best_score[0]] * -1

        # if self.prn == "dwa":
        #     rospy.loginfo("{} {}".format(turtle_vel.linear.x, turtle_vel.angular.z))
        if self.cnt == 0:
            turtle_vel.linear.x, turtle_vel.angular.z = 0, 0
            print("zero")
        
        if self.click_goal.x-0.2 < self.scout_pose.x < self.click_goal.x+0.1 and self.click_goal.y-0.1 < self.scout_pose.y < self.click_goal.y+0.1:
            self.cnt = 0
            dwa_status = True
            status_pub.publish(dwa_status)
            
        
        self.publisher.publish(turtle_vel)
        print(f"turtle vel : {turtle_vel.linear.x}, {turtle_vel.angular.z}")


    def slam_callback(self, data):
        self.scout_pose.x = data.position.x
        self.scout_pose.y = data.position.y
        _, _, self.scout_pose.theta = tf.transformations.euler_from_quaternion(
            [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        # rospy.loginfo("{} {} {}".format(self.scout_pose.x, self.scout_pose.y, self.scout_pose.theta))


    def goal_callback(self, goal):
        self.click_goal.x = goal.point.x
        self.click_goal.y = goal.point.y
        self.click_goal.z = 0
        self.cnt = 1
        print("goal_callback")

    
    @staticmethod
    def preprocessing(data):
        data_array = np.asarray(data.points)
        data_array = data_array[np.logical_and(np.logical_or(data_array[:, 0] > -0.45, data_array[:, 0] < -0.47),
                                               np.abs(data_array[:, 1]) > 0.17)]
        data_array = data_array[np.logical_and(data_array[:, 2] > -0.55, data_array[:, 2] < 0.5)]
        data_array = data_array[data_array[:, 0] > -0.45]
        return data_array

    @staticmethod
    def make_combination(mps, radps):
        line_motions = np.array(mps).reshape(len(mps), 1)
        radps_array = np.delete(np.array(radps), 0)
        rotational_motions = 2 * (line_motions / radps_array) * np.sin(0.5 * radps_array) + 0.05
        local_x_pos = np.concatenate((line_motions, rotational_motions * np.cos(0.5 * radps_array)), axis=1)
        local_y_pos = np.concatenate(
            (np.zeros((len(mps), 1)), rotational_motions * np.sin(0.5 * radps_array)), axis=1)
        local_pos = np.concatenate((np.reshape(local_x_pos, (-1, 1)), np.reshape(local_y_pos, (-1, 1))), axis=1)
        return local_pos

    def create_pos_candidates(self, local_pos):
        rotation_matrix = np.array(
            [[np.cos(self.scout_pose.theta), -np.sin(self.scout_pose.theta)],
             [np.sin(self.scout_pose.theta), np.cos(self.scout_pose.theta)]])
        rotation_trans = np.round_((np.dot(local_pos, rotation_matrix.T)), 4)
        global_x_pos = self.scout_pose.x + np.delete(rotation_trans, 1, axis=1)
        global_y_pos = self.scout_pose.y + np.delete(rotation_trans, 0, axis=1)
        global_pos = np.concatenate((global_x_pos, global_y_pos), axis=1)
        return global_pos

    def evaluate_scores(self, pos_candidates, goal_pos, data_array, local_pos):
        remaining_scores = self.find_remaining_scores(pos_candidates, goal_pos)
        obstacle_distance, obstacle_scores = self.find_obstacle_scores(data_array, local_pos)
        clearance_scores = self.find_clearance_scores(data_array, local_pos)
        scores = 1.6 * remaining_scores + 1.2 * obstacle_scores + clearance_scores
        best_score = np.unravel_index(np.argmax(scores), scores.shape)
        return best_score, obstacle_distance[best_score] <= 0.45

    @staticmethod
    def find_remaining_scores(pos_candidates, goal_pos):
        x = goal_pos[0] - np.delete(pos_candidates, 1, axis=1)
        y = goal_pos[1] - np.delete(pos_candidates, 0, axis=1)
        scores = 1 / np.reshape(np.hypot(x, y), (10, 10))
        norm = np.linalg.norm(scores)
        scores = scores / norm
        return scores

    @staticmethod
    def find_obstacle_scores(data_array, local_pos):
        x = data_array[:, 0] - np.delete(local_pos, 1, axis=1)
        y = data_array[:, 1] - np.delete(local_pos, 0, axis=1)
        distance = np.reshape(np.amin(np.hypot(x, y), axis=1), (10, 10))
        scores = np.where(distance <= 0.45, 0, distance)
        if np.any(scores) != 0:
            norm = np.linalg.norm(scores)
            scores = scores / norm
        return distance, scores

    @staticmethod
    def find_clearance_scores(data_array, local_pos):
        data_array = data_array[np.logical_and(np.logical_and(data_array[:, 0] > -0.1, data_array[:, 0] < 1.0),
                                               np.abs(data_array[:, 1]) > 1.0)]
        x = data_array[:, 0] - np.delete(local_pos, 1, axis=1)
        y = data_array[:, 1] - np.delete(local_pos, 0, axis=1)
        scores = np.reshape(np.mean(np.hypot(x, y), axis=1), (10, 10))
        norm = np.linalg.norm(scores)
        scores = scores / norm
        return scores

def status_callback(s_status):
    global status
    status = s_status.data

    # print(status)


def main():
    rospy.init_node('dwa_test')
    global status

    rospy.Subscriber('status_pub', String, status_callback)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    while not rospy.is_shutdown():

        if status == 'go_goal':
            # print(status)
            driver = DWA(publisher)

            rospy.Subscriber('velodyne_points', PointCloud2, driver.lds_callback)
            rospy.Subscriber('clicked_point', PointStamped, driver.goal_callback)

            dwa_status = True
            status_pub.publish(dwa_status)
            status = 'wait'

        if status == 'dwa_turn':
            print(status)
            turtle_vel = Twist()
            
            current_angle = 0
            # target_angle = np.pi
            target_angle = 2.8
            
            start_time = rospy.Time.now()

            turtle_vel.angular.z = 1.0
            print(turtle_vel)

            while current_angle < target_angle:
                publisher.publish(turtle_vel)

                # 현재 각도 계산
                current_time = rospy.Time.now()
                current_angle = (current_time - start_time).to_sec() * turtle_vel.angular.z

            # 회전 완료 후 로봇을 정지시킴
            print(current_angle)
            turtle_vel.angular.z = 0
            publisher.publish(turtle_vel)

            dwa_status = True
            status_pub.publish(dwa_status)
            status = 'wait'

        if status == 'wait':
            # print(status)
            pass

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
