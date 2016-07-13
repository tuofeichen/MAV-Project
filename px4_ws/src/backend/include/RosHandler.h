#ifndef ROSHANDLER_H_
#define ROSHANDLER_H_

#include "ros/ros.h"
#include <mavros_msgs/BatteryStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

class RosHandler{
public:
	RosHandler();
	~RosHandler();
	void 	 updateCamPos (Matrix4f&); // to pixhawk 
	void 	 updateLpeCam() {_lpe_cam = _lpe; _time_cam = _time; }; // note down new lpe (for next edge calculation)

	Matrix4f getLpe()  				{ return _lpe; };
	double   getTime() 				{ return _time; };

	void 	 getTm(Matrix4f& tm, Matrix<float, 6, 6>& im, double& dt);
	

private: 
	ros::NodeHandle 			_nh;
	ros::Publisher 				_rgbd_slam_pub;
	ros::Subscriber				_lpe_sub;
	ros::Subscriber				_gpe_sub;
	ros::Subscriber				_bat_sub;
	
	bool 						_lpe_valid;    // valid flag
	
	double						_timeout;  
	double 						_time;     // time stamp
	double						_time_cam; 
	Matrix4f 					_lpe;			// curren lpe
	Matrix4f					_lpe_cam; 		// last camera node lpe
	Matrix4f					_lpe2cam;   // transformation matrix

	Vector3f					_rpy;
	Vector3f					_xyz;

	geometry_msgs::PoseStamped 	_rgbd_slam_pos;
	 
	void lpeCallback(const geometry_msgs::PoseStamped pos_read);
	void gpeCallback(const std_msgs::Float64 data);
	void batCallback(const mavros_msgs::BatteryStatus bat);

	
	void q2rpy		(Quaternionf q, float& r, float& p, float& y);
	void rot2rpy	(Matrix3f R,float& r, float& p, float& y);		
	Matrix3f rpy2rot(float r, float p, float y);

};


#endif
