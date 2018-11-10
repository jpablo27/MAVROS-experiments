#include <ros/ros.h>
#include <boost/bind.hpp>
#include <mavros_msgs/PositionTarget.h>
#include <math.h>       /* acos */
#include <geometry_msgs/Point.h>
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


float pose_px,pose_py;


float errx,ux;
float d_errx=0;
float err_prevx=0;
float i_errx=0;

float erry,uy;
float d_erry=0;
float err_prevy=0;
float i_erry=0;

float kp;// =0.5; //0.5 0.07 0.0
float ki;// = 0.0;
float kd;// =0.07 ;

float ref_x;
float ref_y;

float th = 2.0;


void poseGPScb(const geometry_msgs::PoseStamped& msg){


  pose_py = msg.pose.position.y;
  pose_px = msg.pose.position.x;


}

void positionControl(void){
  erry = ref_y - pose_py;
  errx = ref_x - pose_px;

  std::cout << "DY: " << erry << "  DX: " << errx <<std::endl;


  d_errx = errx - err_prevx;
  err_prevx = errx;

  d_erry = erry - err_prevy;
  err_prevy = erry;


  i_errx+=errx;
  ux = errx*kp   +  kd*d_errx  +  ki*i_errx;

  i_erry+=erry;
  uy = erry*kp   +  kd*d_erry  +  ki*i_erry;

  std::cout << "UY: " << uy << "  UX: " << ux <<std::endl<<std::endl;

  if(ux<=-th){
    ux = -th;
  }else if(ux>=th){
    ux = th;
  }

  if(uy<=-th){
    uy = -th;
  }else if(uy>=th){
    uy = th;
  }



  move_parallax.velocity.x= ux;
  move_parallax.velocity.y= uy;

}


int main(int argc, char **argv){

  if (argc!= 6){
    ROS_ERROR("GIMME KP KI KD X Y");
    return -1;
  }else{
    kp = std::atof(argv[1]);
    ki = std::atof(argv[2]);
    kd = std::atof(argv[3]);
    ref_x = std::atof(argv[4]);
    ref_y = std::atof(argv[5]);
  }

  move_parallax.coordinate_frame = 1;
  move_parallax.type_mask = 2503;//2503


      int rate = 60;

      ros::init(argc, argv, "mavros_pos_control");
      ros::NodeHandle n;

      ros::Rate r(rate);


      ros::Subscriber odom_ = n.subscribe("/mavros/local_position/pose",1,poseGPScb);

      ros::Publisher move_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",1);

      while(n.ok()){


        positionControl();
        move_pub.publish(move_parallax);
        ros::spinOnce();
        r.sleep();
      }

}
