#ifndef ROSHANDLER_H_
#define ROSHANDLER_H_

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "Eigen/Core"
#include <Eigen/Dense>
#include <Eigen/Geometry>


using namespace Eigen;

class RosHandler{
public:
	RosHandler();
	~RosHandler();
	void updatePos(Matrix4f&);

private: 
	ros::NodeHandle 			_nh;
	ros::Publisher 				_rgbd_slam_pub;
	geometry_msgs::PoseStamped 	_rgbd_slam_pos; 

};


#endif
