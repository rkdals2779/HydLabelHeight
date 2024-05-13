cmake_minimum_required(VERSION 3.0.2)
project(xarm_move)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  tf2
  tf2_geometry_msgs
  moveit_ros_planning_interface 
  moveit_ros_move_group
  std_msgs
  geometry_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xarm_interface
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs moveit_ros_planning_interface
  DEPENDS EIGEN3
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
)

add_executable(try1 src/try1.cpp)
target_link_libraries(try1 ${catkin_LIBRARIES})
target_link_libraries(try1 ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(try2 src/try2.cpp)
target_link_libraries(try2 ${catkin_LIBRARIES})
target_link_libraries(try2 ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(xarm_move src/xarm_move.cpp)
target_link_libraries(xarm_move ${catkin_LIBRARIES})
target_link_libraries(xarm_move ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(xarm_demo src/xarm_demo.cpp)
target_link_libraries(xarm_demo ${catkin_LIBRARIES})
target_link_libraries(xarm_demo ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})