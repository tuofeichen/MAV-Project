#ifndef Ros_Publisher_H_
#define Ros_Publisher_H_

#include "Eigen/Core"
#include <Eigen/Geometry>

class RosHandler{
public:
	RosHandler();
	~RosHandler();
	void updatePos(Matrix4f TME);

private: 
	ros::NodeHandle 			_nh;
	ros::Publisher 				_rgbd_slam_pub;
	geometry_msgs::PoseStamped 	_rgbd_slam_pos; 

}


#endif
