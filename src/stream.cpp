#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/StreamRate.h>




main(int argc, char** argv){

  ros::init(argc, argv, "parallax_set_rate");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
  mavros_msgs::StreamRate srv;
  srv.request.stream_id = 0;
  srv.request.message_rate = 10;
  srv.request.on_off = 1;


  if(client.call(srv)){
      ROS_INFO("Service called");
  }else{
      ROS_INFO("Failed to call service");
  }
}
