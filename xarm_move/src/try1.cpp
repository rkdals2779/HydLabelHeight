#include <ros/ros.h>
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

using namespace std;
using MoveItErrorCode = moveit::core::MoveItErrorCode;

geometry_msgs::Pose target_pose;
std_msgs::String motion_fin;
string motion_sit;
std::vector<double> kinematic_pose_sub;

int stage = 1;

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
    // void home();
    // void go_catch_scout_side();
    // void go_catch_scout_side_up(int stage);
    void gripper_open();
    void gripper_close();
    // bool gripper_check();
    // void go_catch_cart_side_up();
    // bool task_move(int stage);
    // void up();
    // void clearr();
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


// # home pose 동작하게 한다.
// void xArm::home(){ 
//     cout << "go home" << endl;
//     arm->setNamedTarget("home"); 
//     arm->move();
//     sleep(1);
// }


// # scout쪽 연결봉 잡으러 간다.
// # stage 1은 관절 이동 stage2는 좌표 이동
// void xArm::go_catch_scout_side(){
//     cout << "go catch scout side" << endl;
//     vector<double> currentJointValue = arm->getCurrentJointValues();

//     currentJointValue[0] = 0;
//     currentJointValue[1] = 0.872665;
//     currentJointValue[2] = -0.6562438;
//     currentJointValue[3] = -0.2164208;
//     currentJointValue[4] = 3.14159;
//     arm->setJointValueTarget(currentJointValue);
//     arm->move();
//     sleep(1);
// }


// # scout쪽 위로 연결봉 잡으러 간다. 정교하게 움직이기 위해 2단계로 나눴다.
// void xArm::go_catch_scout_side_up(int stage = 1){
//     cout << "go catch socut side up" << endl;
//     vector<double> currentJointValue = arm->getCurrentJointValues();
//     if(stage == 1){ 
//         currentJointValue[0] = 0;
//         currentJointValue[1] = 0.4276057;
//         currentJointValue[2] = -0.6440265;
//         currentJointValue[3] = 0.2164208;
//         currentJointValue[4] = 3.14159;
//         arm->setJointValueTarget(currentJointValue);
//         arm->move();
//         sleep(1);

//         currentJointValue[0] = 0;
//         currentJointValue[1] = 0.3298672;
//         currentJointValue[2] = -0.4310963;
//         currentJointValue[3] = 0.101229;
//         currentJointValue[4] = 3.14159; //돌아
//         arm->setJointValueTarget(currentJointValue);
//         arm->move();
//         sleep(1);
//     }
//     else{
//         currentJointValue[0] = 0;
//         currentJointValue[1] = 0.3298672;
//         currentJointValue[2] = -0.4310963;
//         currentJointValue[3] = 0.101229;
//         currentJointValue[4] = 3.14159; 
//         arm->setJointValueTarget(currentJointValue);
//         arm->move();
//         sleep(1);

//         currentJointValue[0] = 0;
//         currentJointValue[1] = 0.4276057;
//         currentJointValue[2] = -0.6440265;
//         currentJointValue[3] = 0.2164208;
//         currentJointValue[4] = 0;
//         arm->setJointValueTarget(currentJointValue);
//         arm->move();
//         sleep(1);    
//     }
// }


// # gripper 열기
void xArm::gripper_open(){
    vector<double> currentJointValue2 = gripper->getCurrentJointValues();
    cout << "gripper open" << endl;
    currentJointValue2[0] = 0.00202473;
    gripper->setJointValueTarget(currentJointValue2);
    gripper->move();
    sleep(1);
}


// # gripper 닫기
void xArm::gripper_close(){
    vector<double> currentJointValue2 = gripper->getCurrentJointValues();
    currentJointValue2[0] = 0.570707;
    cout << "gripper close" << endl;
    gripper->setJointValueTarget(currentJointValue2);
    gripper->move();
    sleep(1);
}


// # gripper가 잘 잡았나 확인한다.
// bool xArm::gripper_check(){
//     cout << "gripper_check()" << endl;
//     vector<double> g_check = gripper->getCurrentJointValues();
    
//     if(g_check.front() > 0.58){
//         cout << "g_check false" << endl;
//         return false;
//     }
//     else{
//         cout << "g_check true" << endl;
//         return true;
//     } 
// }


// # 카트쪽 위로 연결봉 잡으러 간다.
// void xArm::go_catch_cart_side_up(){
//     cout << "go catch cart side up" << endl;
//     geometry_msgs::PoseStamped currentPose;
//     currentPose = arm->getCurrentPose();
//     target_pose.position.x = currentPose.pose.position.x;
//     target_pose.position.y = currentPose.pose.position.y;
//     target_pose.position.z = currentPose.pose.position.z + 0.15;
//     target_pose.orientation = currentPose.pose.orientation;
    
//     arm->setPoseTarget(target_pose, "link_tcp");
//     arm->move();
//     sleep(1);
// }


// # 아르코마커 입력받으면 그 곳으로 이동하게 한다.
// bool xArm::task_move(int stage = 1){
//     cout << "task_move()" << endl;
//     if (stage == 2){
//         cout << "stage == 2" << endl;
//         target_pose.position.x = kinematic_pose_sub.at(0);
//         target_pose.position.y = kinematic_pose_sub.at(1);
//         target_pose.position.z = kinematic_pose_sub.at(2) + 0.08;
//         target_pose.orientation.x = kinematic_pose_sub.at(3);
//         target_pose.orientation.y = kinematic_pose_sub.at(4);
//         target_pose.orientation.z = kinematic_pose_sub.at(5);
//         target_pose.orientation.w = kinematic_pose_sub.at(6);
//         arm->setPoseTarget(target_pose, "link_tcp");
//         arm->move();
//         sleep(1);
//     }
//     target_pose.position.x = kinematic_pose_sub.at(0)+0.025;
//     target_pose.position.y = kinematic_pose_sub.at(1);
//     target_pose.position.z = kinematic_pose_sub.at(2);
//     target_pose.orientation.x = kinematic_pose_sub.at(3);
//     target_pose.orientation.y = kinematic_pose_sub.at(4);
//     target_pose.orientation.z = kinematic_pose_sub.at(5);
//     target_pose.orientation.w = kinematic_pose_sub.at(6);
//     arm->setPoseTarget(target_pose, "link_tcp");
    
//     // # 갈 수 있는 경로인지 판단한다. 
//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     bool motion_success = (arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     cout << "task move is ok? : " << motion_success << endl;
//     if (motion_success==true){
//         arm->move();
//         sleep(1);    
//     }
//     return motion_success;
// }


// # 계속 아르코마커 좌표 입력받는다.
// void posecallback(const geometry_msgs::Pose &msg){
//     kinematic_pose_sub.push_back(msg.position.x);
//     kinematic_pose_sub.push_back(msg.position.y);
//     kinematic_pose_sub.push_back(msg.position.z);
//     kinematic_pose_sub.push_back(msg.orientation.x);
//     kinematic_pose_sub.push_back(msg.orientation.y);
//     kinematic_pose_sub.push_back(msg.orientation.z);
//     kinematic_pose_sub.push_back(msg.orientation.w);
// }


// # gripper 때문에 카메라 시야가 가려져서 xArm5를 조금 들어준다.
// void xArm::up(){
//     vector<double> currentJointValue = arm->getCurrentJointValues();
//     currentJointValue[0] = 0;
//     currentJointValue[1] = 0;
//     currentJointValue[2] = 0;
//     currentJointValue[3] = -0.785398;
//     currentJointValue[4] = 0;
//     arm->setJointValueTarget(currentJointValue);
//     arm->move();
//     sleep(1);
// }


// # 입력받은 아르코마커 변수를 지워준다.
// void xArm::clearr(){
//     kinematic_pose_sub.clear();
//     cout << "clear" << endl;
// }


// # 무슨 동작해야하는지 입력받아서 변수에 넣어준다.
// void motioncallback(const std_msgs::String &m){
//     motion_sit = m.data;
// }


// void connection_act(){
//     xarm.gripper_open();
//     r = xarm.task_move();
//     if(r==false){
//         cout << "갈 수 없는 경로이다. 다시" << endl;
//         xarm.clearr();
//         continue;
//         sleep(1);
//     }
//     xarm.gripper_close();
//     r = xarm.gripper_check();
//     if(r==false){
//         cout << "gripper 못 잡았다. 다시" << endl;
//         continue;
//         sleep(1);
//     }
//     xarm.go_catch_cart_side_up();
//     xarm.go_catch_scout_side_up(stage = 1);
//     xarm.go_catch_scout_side();
//     xarm.gripper_open();
//     xarm.go_catch_scout_side_up(stage = 2);
//     xarm.clearr();
//     xarm.home();
// }


// void dismantle_act(){
//     xarm.up();
//     xarm.gripper_open();
//     xarm.go_catch_scout_side_up(stage = 1);
//     xarm.go_catch_scout_side();
//     xarm.gripper_close();
//     r = xarm.gripper_check();
//     if(r==false){
//         cout << "gripper 못 잡았다. 다시" << endl;
//         continue;
//         sleep(1);
//     }
//     xarm.go_catch_scout_side_up(stage = 2);
//     r = xarm.task_move(stage = 2);
//     if(r==false){
//         cout << "갈 수 없는 경로이다. 다시" << endl;
//         xarm.clearr();
//         continue;
//         sleep(1);
//     }
//     xarm.gripper_open();
//     xarm.go_catch_cart_side_up();
//     xarm.home();
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "xarm_move");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // ros::Subscriber sub_xyz = nh.subscribe("camera2_aruco_xyz", 1, posecallback);
    // ros::Subscriber sub_motion = nh.subscribe("xArm_motion_start", 1, motioncallback);
    // ros::Publisher pub_fin = nh.advertise<std_msgs::String>("xArm_motion_fin", 1);
    xArm xarm;
    
    bool r;
    motion_sit = "stay";
    motion_fin.data = "move";
    // xarm.up();
    while (ros::ok())
	{
        if(motion_sit == "xArm_move" & motion_fin.data == "move"){
            cout << "xArm_move" << endl;
            
            // connection_act();

            motion_fin.data = "xArm_move_fin";
            // pub_fin.publish(motion_fin);
            cout << "xArm_move -> motion_fin ->" << motion_fin << endl;
            motion_fin.data = "move2";
        }

        else if(motion_sit == "xArm_move_home" & motion_fin.data == "move2"){
            cout << "xArm_move_home" << endl;
            
            // dismantle_act();

            motion_fin.data = "xArm_move_home_fin";
            // pub_fin.publish(motion_fin);
            cout << "xArm_move_home -> motion_fin ->" << motion_fin << endl;
            motion_fin.data = "stop";
        }
	}
}