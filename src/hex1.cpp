#include <ros/ros.h>
//#include <keyboard/Key.h>
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

bool flag_w = false;
bool flag_a = false;
bool flag_s = false;
bool flag_d = false;

float w, ang;

float err_ang,u;

float d_err,err_prev=0;
float i_err=0;

float kp,ki,kd;//0.003 0.0005 0.001

float ref_x=0.0;
float ref_y=0.0;
/*
void keyupCallback(const keyboard::Key& msg){

  if(msg.code == 119){

    move_parallax.velocity.y=1.0;
  }

  if(msg.code == 115){

      move_parallax.velocity.y=-1.0;
  }
  if(msg.code == 97){

    move_parallax.velocity.x=-1.0;
  }

  if(msg.code == 100){

      move_parallax.velocity.x=1.0;
  }



  if(msg.code == 122){
      move_parallax.yaw=0.5;
  }
  if(msg.code == 120){
      move_parallax.yaw=-0.5;
  }
}

void keydownCallback(const keyboard::Key& msg){
  if(msg.code == 119){

    move_parallax.velocity.y=0;
  }

  if(msg.code == 115){

      move_parallax.velocity.y=0;
  }
  if(msg.code == 97){

    move_parallax.velocity.x=0;
  }

  if(msg.code == 100){

      move_parallax.velocity.x=0;
  }

    if(msg.code == 122){
        move_parallax.yaw=0;
    }
    if(msg.code == 120){
        move_parallax.yaw=0;
    }
}
*/
/*
void poseGPScb(const geometry_msgs::PoseStamped& msg){



  Eigen::Vector3f euler= Eigen::Quaternionf(msg.pose.orientation.w,
      0,
      0,
      msg.pose.orientation.z).toRotationMatrix().eulerAngles(0, 1, 2);


      //std::cout << "EULER: " << euler[2]*180.0/PI << std::endl;

  err_ang = (atan2((0-msg.pose.position.y),(0-msg.pose.position.x)) - euler[2])*180.0/PI;

  std::cout << "ERROR: " << err_ang << std::endl << std::endl;

  d_err = err_ang-err_prev;

  err_prev = err_ang;

  i_err+=err_ang;

  u = err_ang*kp+kd*d_err+ki*err_ang;

  if(u>=1.0){
    u=1.0;
  }else if(u<=-1.0){
    u=-1.0;
  }

  move_parallax.yaw=u;

}
*/
int main(int argc, char **argv){

  //kp = std::stof(argv[1]);
  //kd = std::stof(argv[2]);
  //ki = std::stof(argv[3]);

  //std::cout << "kp: " << kp << "kd: " << kd << "ki: " << ki << std::endl;

  move_parallax.coordinate_frame = 8;
  move_parallax.type_mask = 2503;//2503


      int rate = 10;

      ros::init(argc, argv, "mavros_teleop");
      ros::NodeHandle n;

      ros::Rate r(rate);

      //ros::Subscriber keyup_ = n.subscribe("/keyboard/keydown", 1, keyupCallback);

      //ros::Subscriber keydown_ = n.subscribe("/keyboard/keyup", 1, keydownCallback);

      //ros::Subscriber odom_ = n.subscribe("/mavros/local_position/pose",1,poseGPScb);

      ros::Publisher move_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",1);

      while(n.ok()){
        move_pub.publish(move_parallax);
        ros::spinOnce();
        r.sleep();
      }

}
