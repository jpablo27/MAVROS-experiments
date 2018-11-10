#include <ros/ros.h>
#include <boost/bind.hpp>
#include <mavros_msgs/PositionTarget.h>
#include <math.h>       /* acos */
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <cstdlib>

#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atoi */


#define PI 3.14159265

mavros_msgs::PositionTarget move_parallax;



float err_ang,u;
float d_err,err_prev=0;
float i_err=0;

float kp=0.003;
float ki;
float kd=0.001;

Eigen::Vector3f euler;

float pose_px,pose_py,pose_qw,pose_qz;

void poseGPScb(const geometry_msgs::PoseStamped& msg){

  pose_qw = msg.pose.orientation.w;
  pose_qz = msg.pose.orientation.z;
  pose_py = msg.pose.position.y;
  pose_px = msg.pose.position.x;

  euler= Eigen::Quaternionf(pose_qw,0, 0, pose_qz).toRotationMatrix().eulerAngles(0, 1, 2);

}

void yawControl(float x, float y){
  err_ang = (atan2((y-pose_py),(x-pose_px)) - euler[2])*180.0/PI;

  std::cout << "ERROR: " << err_ang << std::endl << std::endl;

  d_err = err_ang-err_prev;
  err_prev = err_ang;
  i_err+=err_ang;

  u = err_ang*kp+kd*d_err+ki*i_err;

  if(u>=1.0){
    u=1.0;
  }else if(u<=-1.0){
    u=-1.0;
  }

  move_parallax.yaw=u;
}

int main(int argc, char **argv){

  ki=std::stof(argv[1]);
  move_parallax.coordinate_frame = 8;
  move_parallax.type_mask = 2503;//2503


  int rate = 100;

  ros::init(argc, argv, "yaw_control");
  ros::NodeHandle n;

  ros::Rate r(rate);


  ros::Subscriber odom_ = n.subscribe("/mavros/local_position/pose",1,poseGPScb);

  ros::Publisher move_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",1);

  while(n.ok()){
    yawControl(0.0,0.0);
    move_pub.publish(move_parallax);
    ros::spinOnce();
    r.sleep();
  }

}
