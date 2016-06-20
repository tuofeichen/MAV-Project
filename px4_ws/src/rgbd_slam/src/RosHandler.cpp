#include "RosHandler.h"

using namespace Eigen;

RosHandler::RosHandler()
{
	_rgbd_slam_pub = _nh.advertise<geometry_msgs::PoseStamped>("/rgbd_slam/pose",100);
};


RosHandler::~RosHandler()
{

};

void RosHandler::updatePos(Matrix4f& currentTME)
{
		Matrix3f rot_mat  = currentTME.topLeftCorner(3,3);    //  get rotation    matrix
		Quaternionf q(rot_mat); //convert to quaternion
		Vector3f trans_vec  = currentTME.topRightCorner(3,1); //  get translation vector

		_rgbd_slam_pos.pose.position.x = trans_vec(0);
		_rgbd_slam_pos.pose.position.y = trans_vec(1);
		_rgbd_slam_pos.pose.position.z = trans_vec(2);

		_rgbd_slam_pos.pose.orientation.w = q.w();
		_rgbd_slam_pos.pose.orientation.x = q.x();
		_rgbd_slam_pos.pose.orientation.y = q.y();
		_rgbd_slam_pos.pose.orientation.z = q.z();
		_rgbd_slam_pub.publish(_rgbd_slam_pos);


}

