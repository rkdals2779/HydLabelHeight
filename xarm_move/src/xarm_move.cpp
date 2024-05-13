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
std::vector<double> kinematic_pose_sub;
std_msgs::String status;
std_msgs::Bool arm_status;

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
    void home();
    void zero_home();
    void gripper_open();
    void gripper_close();
    void gripper_spin_90();
    void gripper_spin_180();
    void gripper_horizon();
    void task_move();
    void handle_move();
    void xarm_up();
    void xarm_down();
    void ddddd();
    // void cur_grip();
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

// void xArm::cur_grip(){
//     vector<double> currentJointValue2 gripper->getCurrentJointValues();   // Gripper 조절 변수
    
//     printf("\ngripper : %f", currentJointValue2[0]);
    
// }

void xArm::ddddd(){
    geometry_msgs::PoseStamped currentPose; // 현재 Pose를 담을 변수s

    currentPose = arm->getCurrentPose();    // 현재 Pose

    target_pose.position.x = 0.60;
    target_pose.position.y = 0;
    target_pose.position.z = 0.25;
    target_pose.orientation = currentPose.pose.orientation;

    arm->setPoseTarget(target_pose, "link_tcp");
    arm->move();

    // 1차 이동하는 좌표값
    printf("\nx : %f\ny : %f\nz : %f\n", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // 잡을 수 있는 위치인지 판별
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool motion_success = (arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout << "task move is ok? : " << motion_success << endl;
    if (motion_success==true){
        // 넣어진 물체의 목표값을 set하고 이동
        target_pose.position.z = 0;

        arm->setPoseTarget(target_pose, "link_tcp");
        arm->move();
    }
}

void xArm::gripper_horizon(){
    cout << "gripper_horizon" << endl;

    vector<double> currentJointValue = arm->getCurrentJointValues();    // 현재 Joint

    // 잡기 편하게 Gripper 회전
    currentJointValue[3] = 0.0;

    arm->setJointValueTarget(currentJointValue);
    arm->move();
}

void xArm::handle_move(){
    cout << "handle_move" << endl;

    geometry_msgs::PoseStamped currentPose; // 현재 Pose를 담을 변수s

    currentPose = arm->getCurrentPose();    // 현재 Pose

    // 물체를 잡을 때 걸리지 않게, 물체의 x, y와 Home에서의 z값으로 이동
    target_pose.position.x = 0.32;
    target_pose.position.y = kinematic_pose_sub.at(1) + 0.015;
    target_pose.position.z = kinematic_pose_sub.at(2);
    // target_pose.position.z = currentPose.pose.position.z;
    target_pose.orientation = currentPose.pose.orientation;
    // 1차 이동하는 좌표값
    printf("\nx : %f\ny : %f\nz : %f\n", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // 잡을 수 있는 위치인지 판별
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool motion_success = (arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout << "task move is ok? : " << motion_success << endl;
    if (motion_success==true){
        // 넣어진 물체의 목표값을 set하고 이동
        arm->setPoseTarget(target_pose, "link_tcp");
        arm->move();

        // 걸리지 않게 Home의 x값을 넣었던 값을, 물체의 x로 넣은 후 이동
        target_pose.position.x = kinematic_pose_sub.at(0);

        arm->setPoseTarget(target_pose, "link_tcp");
        arm->move();
    }

    // 2차 이동하는 좌표값
    printf("\nx : %f\ny : %f\nz : %f\n", target_pose.position.x, target_pose.position.y, target_pose.position.z);
}

// 물체까지 이동
void xArm::task_move(){
    cout << "task_move" << endl;

    geometry_msgs::PoseStamped currentPose; // 현재 Pose를 담을 변수s

    currentPose = arm->getCurrentPose();    // 현재 Pose

    // 물체를 잡을 때 걸리지 않게, 물체의 x, y와 Home에서의 z값으로 이동
    target_pose.position.x = kinematic_pose_sub.at(0) - 0.03;
    target_pose.position.y = kinematic_pose_sub.at(1);
    target_pose.position.z = currentPose.pose.position.z;
    target_pose.orientation = currentPose.pose.orientation;
    // 1차 이동하는 좌표값
    printf("\nx : %f\ny : %f\nz : %f\n", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // 잡을 수 있는 위치인지 판별
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool motion_success = (arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout << "task move is ok? : " << motion_success << endl;
    if (motion_success==true){
        // 넣어진 물체의 목표값을 set하고 이동
        arm->setPoseTarget(target_pose, "link_tcp");
        arm->move();

        // 걸리지 않게 Home의 z값을 넣었던 값을, 물체의 z로 넣은 후 이동
        target_pose.position.z = kinematic_pose_sub.at(2);

        arm->setPoseTarget(target_pose, "link_tcp");
        arm->move();
    }

    // 2차 이동하는 좌표값
    printf("\nx : %f\ny : %f\nz : %f\n", target_pose.position.x, target_pose.position.y, target_pose.position.z);
}

void xArm::zero_home(){
    cout << "zero_home" << endl;

    arm->setNamedTarget("home"); 
    arm->move();

    // geometry_msgs::PoseStamped currentPose; // 현재 Pose를 받을 변수
    // currentPose = arm->getCurrentPose();    // 현재 Pose

    // target_pose.position.x = 0.3;
    // target_pose.position.y = 0;
    // target_pose.position.z = 0.3;
    // target_pose.orientation.x = 0; // 목표 orientation -> 현재 orientation
    // target_pose.orientation.y = 0;
    // target_pose.orientation.z = 0;
    // target_pose.orientation.w = 1;


    // arm->setPoseTarget(target_pose, "link_tcp");
    // arm->move();

    // vector<double> currentJointValue = arm->getCurrentJointValues();    // 현재 Joint
    // currentJointValue[4] = 0; // Gripper 제자리

    // arm->setJointValueTarget(currentJointValue);
    // arm->move();
}

// home pose
void xArm::home(){
    cout << "go home" << endl;

    geometry_msgs::PoseStamped currentPose; // 현재 Pose를 받을 변수
    currentPose = arm->getCurrentPose();    // 현재 Pose

    target_pose.position.x = 0.3;
    target_pose.position.y = 0;
    target_pose.position.z = 0.3;
    target_pose.orientation = currentPose.pose.orientation;
    // target_pose.orientation.x = 1;
    // target_pose.orientation.y = 0;
    // target_pose.orientation.z = 0;
    // target_pose.orientation.w = 0;   // 목표 orientation -> 현재 orientation

    arm->setPoseTarget(target_pose, "link_tcp");
    arm->move();

    vector<double> currentJointValue = arm->getCurrentJointValues();    // 현재 Joint
    currentJointValue[4] = 0; // Gripper 제자리

    arm->setJointValueTarget(currentJointValue);
    arm->move();
    
    // printf("\nx : %f\ny : %f\nz : %f\nw: %f\n", currentPose.pose.orientation.x, currentPose.pose.orientation.w, currentPose.pose.orientation.z, currentPose.pose.orientation.w);
}

// gripper spin 90 code
void xArm::gripper_spin_90(){
    cout << "gripper_spin_90" << endl;

    vector<double> currentJointValue = arm->getCurrentJointValues();    // 현재 Joint

    // 잡기 편하게 Gripper 회전
    currentJointValue[4] = 1.72;

    arm->setJointValueTarget(currentJointValue);
    arm->move();
}

// gripper spin 180 code
void xArm::gripper_spin_180(){
    cout << "gripper_spin_180" << endl;

    vector<double> currentJointValue = arm->getCurrentJointValues();    // 현재 Joint

    // 잡기 편하게 Gripper 회전
    currentJointValue[4] = 3.05;

    arm->setJointValueTarget(currentJointValue);
    arm->move();
}

// gripper 열기
void xArm::gripper_open(){
    cout << "gripper open" << endl;

    vector<double> currentJointValue2 = gripper->getCurrentJointValues();   // Gripper 조절 변수
    currentJointValue2[0] = 0.00202473;

    gripper->setJointValueTarget(currentJointValue2);
    gripper->move();
}

// gripper 닫기
void xArm::gripper_close(){
    cout << "gripper close" << endl;

    vector<double> currentJointValue2 = gripper->getCurrentJointValues();   // Gripper 조절 변수
    currentJointValue2[0] = 0.63700;
    // printf("\ngripper : %f", currentJointValue2[0]);

    gripper->setJointValueTarget(currentJointValue2);
    gripper->move();
}

// handle catch after up
void xArm::xarm_up(){
    cout << "xarm_up" << endl;

    geometry_msgs::PoseStamped currentPose; // 현재 Pose를 받을 변수
    currentPose = arm->getCurrentPose();    // 현재 Pose

    target_pose.position.z = 0.00;
    target_pose.orientation = currentPose.pose.orientation; // 목표 orientation -> 현재 orientation

    arm->setPoseTarget(target_pose, "link_tcp");
    arm->move();
}

// handle up after down
void xArm::xarm_down(){
    cout << "xarm_down" << endl;

    geometry_msgs::PoseStamped currentPose; // 현재 Pose를 받을 변수
    currentPose = arm->getCurrentPose();    // 현재 Pose

    target_pose.position.x = 0.35;
    target_pose.position.y = 0;
    target_pose.position.z = -0.07;
    target_pose.orientation = currentPose.pose.orientation; // 목표 orientation -> 현재 orientation

    arm->setPoseTarget(target_pose, "link_tcp");
    arm->move();
}

// 이동할 Aruco Marker의 좌표값
void posecallback(const geometry_msgs::Pose &msg){
    kinematic_pose_sub.clear();

    kinematic_pose_sub.push_back(msg.position.x);
    kinematic_pose_sub.push_back(msg.position.y);
    kinematic_pose_sub.push_back(msg.position.z);
}

// X-Arm status sub
void status_callback(const std_msgs::String::ConstPtr& msg)
{
    status.data = msg->data;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "xarm_move");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    spinner.start();

    int cnt = 0;

    ros::Subscriber sub_xyz = nh.subscribe("arm_point", 1, posecallback);  // Aruco Marker 좌표값
    ros::Subscriber status_sub = nh.subscribe("status_pub", 1, status_callback);    // X-Arm status

    ros::Publisher arm_status_pub = nh.advertise<std_msgs::Bool>("arm_status", 1);

    xArm xarm;

    // xarm.home();
    // xarm.gripper_spin_90();
    // xarm.gripper_close();
    // xarm.gripper_spin_180();

    
    // xarm.gripper_horizon();
    // xarm.ddddd();
    // xarm.cur_grip();

    status.data = "";
    arm_status.data = false;

    xarm.home();

    // return 0;

    while (ros::ok())
    {
        if (status.data == "grab")
        {
            if (cnt == 0)
            {   
                cnt = 1;
                xarm.home();
                xarm.gripper_open();
                xarm.gripper_spin_90();
                xarm.task_move();
                xarm.gripper_close();
                xarm.home();

                arm_status.data = true;
                arm_status_pub.publish(arm_status);
                status.data = "wait";
            }

            else
            {
                status.data = "wait";
            }
        }

        if (status.data == "release")
        {
            if (cnt == 1)
            {
                cnt = 0;
                xarm.home();
                xarm.ddddd();
                xarm.gripper_open();
                xarm.home();
                xarm.gripper_close();

                arm_status.data = true;
                arm_status_pub.publish(arm_status);
                status.data = "wait";
            }

            else
            {
                status.data = "wait";
            }
        }

        if (status.data == "cart_on")
        {
            if (cnt == 0)
            {
                cnt = 1;
                xarm.home();
                xarm.gripper_open();
                xarm.handle_move();
                xarm.gripper_close();
                xarm.xarm_up();
                xarm.gripper_spin_180();
                xarm.xarm_down();
                xarm.gripper_open();
                xarm.gripper_close();
                xarm.home();

                arm_status.data = true;
                arm_status_pub.publish(arm_status);
                status.data = "wait";
            }

            else
            {
                status.data = "wait";
            }
        }

        ros::spinOnce();
    }
}