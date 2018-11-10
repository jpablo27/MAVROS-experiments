#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/StreamRate.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>

//#include <Eigen/Dense>
bool poseflag=false;

bool controlPosition = false;
bool controlOrientation = false;

#define PI 3.14159265

mavros_msgs::PositionTarget move_parallax;

//control functions
void YawControl(float x, float y),PositionControl(float x, float y);
void YawControlRad(void);

//YawControlVariables
geometry_msgs::TwistStamped YawVel;
float pose_px,pose_py,pose_qw,pose_qz;
float err_ang=5,ua,ud;
float d_err,err_prev=0;
float i_err=0;
float kp_ang=0.01;
float ki_ang=0;
float kd_ang=0.00003;

//PositionControlVariables
float kp_dist=0.5;
float ki_dist=0.0;
float kd_dist=0.07;
float errx,erry;
float d_errd=0;
float err_prevd=0;

float ref_x;
float ref_y;
float yaw_ref;
float ref_abs;

float err_dist=5;

Eigen::Vector3f euler;

float thd = 1.0;
float tha = 0.6;

float alpha;

//YawControlCallbacks
void poseGPScb(const geometry_msgs::PoseStamped& msg);


main(int argc, char** argv){

  if(argc == 3){
    ref_x = std::atof(argv[1]);
    ref_y = std::atof(argv[2]);
  }else if (argc == 2) {
    /* code */
    yaw_ref = std::atof(argv[1]);
  }else{

      ROS_ERROR("GIMME X Y");
      return -1;

  }
  move_parallax.coordinate_frame = 8;
  move_parallax.type_mask = 4039;//2503

  int rate = 3;

  ros::init(argc, argv, "wp_follow");
  ros::NodeHandle n;

  ros::Rate r(rate);

  ros::Subscriber odom_ = n.subscribe("/mavros/local_position/pose",1,poseGPScb);
  ros::Publisher move_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",1);
  ros::Publisher move_Yaw = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);

  while (poseflag==false) { //wait for poseCB
    r.sleep();
    ros::spinOnce();
  }

  ref_abs = alpha + yaw_ref;//fixed

  while(ros::ok()){




    if((err_dist>0.3)){
      controlOrientation = false;
      YawControl(ref_x,ref_y);
    }else{
      YawVel.twist.angular.z =0;
      controlOrientation = true;
    }

    if((err_ang<1.0)&&(err_ang>-1.0)){
      controlPosition = false;
      PositionControl(ref_x,ref_y);
    }else{
      move_parallax.velocity.y=0;
      controlPosition = true;
    }
    
    YawVel.header.stamp = move_parallax.header.stamp = ros::Time::now();
    move_Yaw.publish(YawVel);
    move_pub.publish(move_parallax);
    ros::spinOnce();
    r.sleep();

    if(controlOrientation){//&&controlPosition){
      std::cout << "LLegué "<< std::endl;
      break;
    }
  }

  return 0;
}


void YawControl(float x, float y){
  err_ang = (atan2((y-pose_py),(x-pose_px)) - euler[2]);
  err_ang = atan2(sin(err_ang),cos(err_ang))*180/PI;

  d_err = err_ang-err_prev;
  err_prev = err_ang;
  i_err+=err_ang;

  ua = kp_ang*err_ang  +  kd_ang*d_err  +  ki_ang*i_err;

  if(ua>=tha){
    ua=tha;
  }else if(ua<=-tha){
    ua=-tha;
  }

  if((err_ang<0.1)&&(err_ang>-0.1)){
    ua = 0.0;
    controlOrientation = true;
  }
  YawVel.twist.angular.z=ua;
}

void YawControlRad(void){

  err_ang = (ref_abs - alpha);
  err_ang = atan2(sin(err_ang),cos(err_ang))*180/PI;

  d_err = err_ang-err_prev;
  err_prev = err_ang;
  i_err+=err_ang;

  ua = kp_ang*err_ang  +  kd_ang*d_err  +  ki_ang*i_err;

  if(ua>=tha){
    ua=tha;
  }else if(ua<=-tha){
    ua=-tha;
  }

  if((err_ang<0.1)&&(err_ang>-0.1)){
    ua = 0.0;
    controlOrientation = true;
  }
  YawVel.twist.angular.z=ua;

}


void PositionControl(float x, float y){
  erry = y - pose_py;
  errx = x - pose_px;

  err_dist = sqrt(pow(erry,2)+pow(errx,2));

  d_errd = err_dist - err_prevd;
  err_prevd = err_dist;

  ud = kp_dist*err_dist + kd_dist*d_errd;

  if(ud<=0.0){
    ud = 0.0;
  }else if(ud>=thd){
    ud = thd;
  }

  if(err_dist<0.1){
    ud = 0.0;
    controlPosition = true;
  }

  move_parallax.velocity.y = ud;
}

void poseGPScb(const geometry_msgs::PoseStamped& msg){

  pose_qw = msg.pose.orientation.w;
  pose_qz = msg.pose.orientation.z;
  pose_py = msg.pose.position.y;
  pose_px = msg.pose.position.x;

  euler= Eigen::Quaternionf(pose_qw,0, 0, pose_qz).toRotationMatrix().eulerAngles(0, 1, 2);
  alpha = euler[2];
  poseflag = true;
}
