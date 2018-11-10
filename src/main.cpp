#include <cstdlib>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/StreamRate.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Geometry>

#define PI 3.141592
geometry_msgs::PoseStamped current_state;
mavros_msgs::PositionTarget move_parallax;
geometry_msgs::TwistStamped YawVel;


float err_YA=5;
float uYA;
float pose_pxYA,pose_pyYA,pose_qwYA,pose_qzYA;
Eigen::Vector3f eulerYA;
float kp_YA=0.01;
float ki_YA=0;
float kd_YA=0.00003;
float thYA = 0.6;
float d_errYA,err_prevYA=0;
float i_errYA=0;
bool poseflag=false;
void poseGPScb(const geometry_msgs::PoseStamped& msg);


void YawControlByAngle(float refAngle){//Input between -PI and PI
  err_YA = (refAngle - eulerYA[2]);

  err_YA = atan2(sin(err_YA),cos(err_YA))*180/PI;
std::cout << "POSICIÓN ACTUAL : " << eulerYA[2]*180/PI << std::endl << std::endl;
  std::cout << "ERROR ANGULAR : " << err_YA << std::endl << std::endl;

  d_errYA = err_YA-err_prevYA;
  err_prevYA = err_YA;
  i_errYA+=err_YA;

  uYA = kp_YA*err_YA  +  kd_YA*d_errYA  +  ki_YA*i_errYA;

  if(uYA>=thYA){
    uYA=thYA;
  }else if(uYA<=-thYA){
    uYA=-thYA;
  }
  std::cout << "SALIDA: " << uYA << std::endl << std::endl;

  if((err_YA<0.1)&&(err_YA>-0.1)){
    uYA = 0.0;
  }
  YawVel.twist.angular.z=uYA;
}


int main(int argc, char **argv)
{

  if(argc != 2){
    return -1;
  }





    int rate = 10;

    ros::init(argc, argv, "mavros_takeoff");
    ros::NodeHandle n;

    ros::Rate r(rate);

    move_parallax.coordinate_frame = 8;
    move_parallax.type_mask = 2503; // Ignore only accelerations


    //l_wp.yaw = std::atof(argv[1]);
    //l_wp.yaw_rate = std::atof(argv[2]);



    ros::Publisher move_pub = n.advertise<mavros_msgs::PositionTarget>
    ("/mavros/setpoint_raw/local",1);

    ros::Publisher move_Yaw = n.advertise<geometry_msgs::TwistStamped>
    ("/mavros/setpoint_velocity/cmd_vel",1);

    ////////////////////////////////////////////
    ////////////////rate change/////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient client = n.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    mavros_msgs::StreamRate srvr;
    srvr.request.stream_id = 0;
    srvr.request.message_rate = 10;
    srvr.request.on_off = 1;
    if(client.call(srvr)){
        ROS_INFO("Service called");
    }else{
        ROS_ERROR("Failed to call service");
        return -1;
    }

    ros::Subscriber state_sub = n.subscribe
    ("/mavros/local_position/pose", 1, poseGPScb);
    ////////////////////////////////////////////
    /////////////////GUIDED/////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    if(cl.call(srv_setMode)){
        ROS_ERROR("setmode send ok %d value:", srv_setMode.response.mode_sent);
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_ERROR("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }

    ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 3;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_ERROR("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }

    //Esperar a que el dron alcance la altura deseada
    /*while( !(current_state.pose.position.z>=(3-0.05)) ){
      r.sleep();
      ros::spinOnce();
    }
    ROS_ERROR("3m");*/


    ////////////////////////////////////////////
    /////////////////DO STUFF///////////////////
    ////////////////////////////////////////////
    //sleep(5);
    while (poseflag==false) { //wait for poseCB
      r.sleep();
      ros::spinOnce();
    }

    float goalAngle = std::atof(argv[1]) + eulerYA[2];

    goalAngle = atan2(sin(goalAngle),cos(goalAngle));


    while (((err_YA>0.1)||(err_YA<-0.1))&&ros::ok()) {


        YawControlByAngle(goalAngle);
        std::cout <<"GOAL: "<< goalAngle*180/PI << std::endl;


        YawVel.header.stamp = ros::Time::now();

        move_Yaw.publish(YawVel);





      r.sleep();
      ros::spinOnce();

    }



    sleep(5);

    ////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    /*ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 3;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }
*/
    while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}

void poseGPScb(const geometry_msgs::PoseStamped& msg){

  current_state = msg;

  pose_qwYA = msg.pose.orientation.w;
  pose_qzYA = msg.pose.orientation.z;
  pose_pyYA = msg.pose.position.y;
  pose_pxYA = msg.pose.position.x;

  eulerYA= Eigen::Quaternionf(pose_qwYA,0, 0, pose_qzYA).toRotationMatrix().eulerAngles(0, 1, 2);

  poseflag = true;

}
