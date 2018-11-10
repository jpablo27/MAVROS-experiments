cmake_minimum_required(VERSION 2.8.3)
project(parallax)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros
)


catkin_package()


include_directories(${catkin_INCLUDE_DIRS})
include_directories(include include)


add_definitions(-std=c++0x -g -O3)


add_executable(parallax_node src/main.cpp)
add_executable(parallax_trajecttory src/maintrajextory.cpp)
#add_executable(parallax_teleop src/hex1.cpp)
add_executable(parallax_yaw src/YawControl.cpp)
add_executable(parallax_pos src/Position.cpp)
add_executable(gaz_sim src/posSim.cpp)
add_executable(set_rate src/stream.cpp)
add_executable(follow_wp src/wP.cpp)

target_link_libraries(parallax_node ${catkin_LIBRARIES})
target_link_libraries(parallax_trajecttory ${catkin_LIBRARIES})
#target_link_libraries(parallax_teleop ${catkin_LIBRARIES})
target_link_libraries(parallax_yaw ${catkin_LIBRARIES})
target_link_libraries(parallax_pos ${catkin_LIBRARIES})
target_link_libraries(gaz_sim ${catkin_LIBRARIES})
target_link_libraries(set_rate ${catkin_LIBRARIES})
target_link_libraries(follow_wp ${catkin_LIBRARIES})
