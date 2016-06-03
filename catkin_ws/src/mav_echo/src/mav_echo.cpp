#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
//#include "mavros_msgs/local_positoin/pose.h"

float x = 0, y = 0, z = 0;

void mavCallback(const geometry_msgs::PoseStamped pose)
{
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    z = pose.pose.position.z;
}


int main(int argc, char ** argv)
{
  ros::init(argc,argv,"talker");
  ros::NodeHandle n;
  ros::Publisher  mav_echo_pub = n.advertise<geometry_msgs::Vector3>("mav_echo",100);
  ros::Subscriber mav_echo_sub = n.subscribe("/mavros/local_position/pose",1000,mavCallback);
  geometry_msgs::Vector3 echo_msg;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ros::spinOnce();
    echo_msg.x = x;
    echo_msg.y = y;
    echo_msg.z = z;
    mav_echo_pub.publish(echo_msg);
    loop_rate.sleep();
  }
}
