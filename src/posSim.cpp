#include <ros/ros.h>
//#include <keyboard/Key.h>
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
float i_prevx=0;

float erry,uy;
float d_erry=0;
float err_prevy=0;
float i_erry=0;
float i_prevy=0;


float kp ; //0.5 0.07 0.0
float ki ;
float kd ;

float ref_x=0.0;
float ref_y=0.0;

float th = 0.3;

float t_1 = 0; //= ros::Time::now();
float T;
double t;

void poseCB(const geometry_msgs::PoseStamped& msg){
  pose_px = msg.pose.position.x;
  pose_py = msg.pose.position.y;
}

void positionControl(void){
  erry = ref_y - pose_py;
  errx = ref_x - pose_px;

  t = ros::Time::now().toSec();

  T=t-t_1;
  t_1= t;

  d_errx = (errx - err_prevx)/T;

  d_erry = (erry - err_prevy)/T;

  std::cout << "DY: " << float(ros::Time::now().toSec())<< "  DX: " << errx <<std::endl;

  i_errx= i_prevx + (float(T)/2)*(errx+err_prevx);
  i_prevx = i_errx;
  err_prevx = errx;
  ux = errx*kp   +  kd*d_errx  +  ki*i_errx;

  i_erry =i_prevy + (T/2)*(erry+err_prevy) ;
  i_prevy = i_erry;
  err_prevy = erry;
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

  if(argc != 4){
    ROS_ERROR("usage pkg node kp ki kd");
    return -1;
  }

  kp = std::atof(argv[1]);
  ki = std::atof(argv[2]);
  kd = std::atof(argv[3]);


  move_parallax.coordinate_frame = 8;
  move_parallax.type_mask = 2503;//2503


      int rate = 60;

      ros::init(argc, argv, "sim_control");
      ros::NodeHandle n;

      ros::Rate r(rate);


      ros::Subscriber odom_ = n.subscribe("/mavros/local_position/pose",1,poseCB);

      ros::Publisher move_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",1);

      while(n.ok()){
        positionControl();
        move_pub.publish(move_parallax);
        ros::spinOnce();
        r.sleep();
      }

}
