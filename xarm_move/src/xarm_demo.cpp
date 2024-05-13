#include <ros/ros.h>
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <unistd.h>

using namespace std;
using MoveItErrorCode = moveit::core::MoveItErrorCode;

geometry_msgs::Pose target_pose;
std_msgs::String motion_fin;
string motion_sit;
std::vector<double> kinematic_pose_sub;
std_msgs::String status;

class xArm
{
private:
    ros::NodeHandle nh;
    std::string planning_group_name;
    std::string planning_group_name2;
    moveit::planning_interface::MoveGroupInterface* arm;
    moveit::planning_interface::MoveGroupInterface* gripper;

public:
    xArm();
    ~xArm();
    std::vector<double> currentJointValue;
    std::vector<double> currentJointValue2;
    void task_move();
};


xArm::xArm()
:nh("")
{
    planning_group_name = "xarm5";
    planning_group_name2 = "xarm_gripper";
    arm = new moveit::planning_interface::MoveGroupInterface(planning_group_name);
    gripper = new moveit::planning_interface::MoveGroupInterface(planning_group_name2);
    vector<double> currentJointValue = arm->getCurrentJointValues();
    // # 속도 제어
    arm->setGoalJointTolerance(0.001);
    arm->setMaxAccelerationScalingFactor(0.6);
    arm->setMaxVelocityScalingFactor(0.45);
}


xArm::~xArm()
{
	if (ros::isStarted()) 
	{
		ros::shutdown();
		ros::waitForShutdown();
	}
}

void xArm::task_move(){
    cout << "task_move" << endl;

    vector<double> currentJointValue = arm->getCurrentJointValues();
    currentJointValue[4] = 0;
    arm->setJointValueTarget(currentJointValue);
    arm->move();

    geometry_msgs::PoseStamped currentPose;
    currentPose = arm->getCurrentPose();
    target_pose.orientation = currentPose.pose.orientation;
    printf("\nx : %f\ny : %f\nz : %f\nw : %f\n", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

    target_pose.position.x = -0.8;
    target_pose.position.y = 0.5;
    target_pose.position.z = -0.5;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool motion_success = (arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout << "task move is ok? : " << motion_success << endl;
    if (motion_success==true){
        printf("task_move_success");
        arm->setPoseTarget(target_pose, "link_tcp");
        arm->move();
        sleep(1);    
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "xarm_move");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    spinner.start();

    xArm xarm;
    
    xarm.task_move();
}   