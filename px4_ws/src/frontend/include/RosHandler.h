#ifndef ROSHANDLER_H_
#define ROSHANDLER_H_

#include "ros/ros.h"
#include <mavros_msgs/BatteryStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include "frontend/MavState.h"


#include <std_msgs/Float64.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;


/**
 * @class RosHandler RosHandler.h "RosHandler.h"
 * @brief The RosHandler class interfaces with mavros and other ROS nodes (read and write)
 */

class RosHandler{
public:
	/**
	 * @brief constructor
	 *
	 * initialize all ROS related and data structures needed
	 */
	RosHandler();
	/**
	 * @brief Destructor
	 */
	~RosHandler();
	/**
	 * @brief publish vslam position estimate to mavros
	 */
	void 	 updateCamPos (double, Matrix4f);
	/**
	 * @brief store new LPE estimate in the vector `nodes`
	 */
	void 	 updateLpeLastPose(int id);
	/**
	 * @brief update object pose (related to the camera)
	 *
	 * Point.x (centroid x location (value from 0 to Image Width))
	 * Point.y (centroid y location (value from 0 to Image Height))
	 * Point.z (Distance from the object (mm))
	 */
	void 	 updateObjPos  (geometry_msgs::Point);
	/**
	 * @brief update wall pose (related to the camera)
	 * Point.x (Angle to wall (rad))
	 * Point.y (Angle to wall (deg))
	 * Point.z (Distance from the wall (mm))
	 */
	void 	 updateWallPos (geometry_msgs::Point);
	/**
	 * @brief update wall pose (related to the camera)
	 * Point.x (number of failed obstacle detected)
	 * Point.y (minimum nonzero distance in the scene)
	 * Point.z (average distance of obstacles in the scene)
	 */
	void 	 updateObstacleDistance(geometry_msgs::Point obsDist);

	/**
	 * @brief return most recent LPE
	 */
	Matrix4f getLpe();
	/**
	 * @brief fuse the input transform with the LPE estimate transform
	 * (from, to is the pose graph node id)
	 */
	Matrix4f fuseLpeTm(Matrix4f,int from, int to);

	/**
	 * @brief fuse the rotaiona of input transform with the LPE estimate transform
	 *
	 */
	Matrix3f fuseRpy(Matrix3f vRot);

	/**
	 * @brief return LPE transform from -> to
	 *
	 */
	Matrix4f 	 getTmFromIdtoId(int from, int to);
	bool 	 getTakeoffFlag(){ return _is_takeoff;};
	bool 	 getArmFlag(){ return _is_arm;};

private:
	ros::NodeHandle 			_nh;
	ros::Publisher 				_rgbd_slam_pub; // go to mavros
	ros::Publisher				_object_pub;    // go to px4_offboard
	ros::Publisher				_wall_pub;	// go to px4_offboard
	ros::Publisher				_obst_pub;	// go to px4_offboard

	ros::Subscriber 			_state_sub;
	ros::Subscriber				_px4_offboard_sub;
	ros::Subscriber				_lpe_sub;
	ros::Subscriber				_flow_valid_sub;
	ros::Subscriber				_bat_sub;

	bool 						_lpe_valid;    // valid flag

	bool 						_is_takeoff;
	bool 						_is_land;
	bool						_is_fail;
	bool 						_is_arm; // note down arming instance is important for some handling
	double						_timeout;
	double 						_time;     	// time stamp


	std::vector<Matrix4f> _lpe_nodes;
	// note down the stream of lpe

	Matrix4f 						_lpe;									// current lpe
	Vector3f					_xyz;

	geometry_msgs::PoseStamped 	_rgbd_slam_pos;

	void lpeCallback(const geometry_msgs::PoseStamped pos_read);
	void flowValidCallback(const std_msgs::Float64 data);
	void batCallback(const mavros_msgs::BatteryStatus bat);


// rotation utilities
	void q2rpy		(Quaternionf q, float& r, float& p, float& y);
	void rot2rpy	(Matrix3f R,float& r, float& p, float& y);
	Matrix3f rpy2rot(float r, float p, float y);

	void stateCallback(const frontend::MavState state)
	{
	_is_takeoff = state.takeoff;
	_is_land 	= state.land;
	_is_fail 	= state.failsafe;
	_is_arm   = state.arm;
	};


};


#endif
