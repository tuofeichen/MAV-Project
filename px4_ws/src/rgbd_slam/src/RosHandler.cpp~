#include "RosHandler.h"
#include <math.h>

using namespace Eigen;
using namespace std;

RosHandler::RosHandler()
{
	_rgbd_slam_pub = _nh.advertise<geometry_msgs::PoseStamped>("/rgbd_slam/pose",100);
	_lpe_sub = _nh.subscribe("/mavros/local_position/pose",100,&RosHandler::lpeCallback,this);
};


RosHandler::~RosHandler()
{
	ros::shutdown();
};


void RosHandler::lpeCallback(const geometry_msgs::PoseStamped pos_read)
{
	Quaternionf q (pos_read.pose.orientation.w, pos_read.pose.orientation.x, pos_read.pose.orientation.y,pos_read.pose.orientation.z);
	q.normalize();

//Debug
	// Vector3f rpy; 
	// Matrix3f rot = q.matrix();
	// q2rpy(q,rpy(0),rpy(1),rpy(2));
	// rot2rpy(rot,rpy(0),rpy(1),rpy(2));

	_lpe.setZero(); // clean buffer
	_lpe.topLeftCorner(3,3)  = q.matrix(); // update rotation matrix
	_lpe.topRightCorner(3,1) << pos_read.pose.position.x, pos_read.pose.position.y, 	pos_read.pose.position.z;  // update translation vector

}

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

void RosHandler::q2rpy(Quaternionf q, float& r, float& p, float& y)
{
	r = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(q.x()*q.x()+q.y()*q.y()));
	p = asin(2*(q.w()*q.y()-q.z()*q.x()));
	y = atan2(2*(q.w()*q.z()+q.y()*q.x()), 1-2*(q.z()*q.z()+q.y()*q.y()));

}

void RosHandler::rot2rpy(Matrix3f R,float& r, float& p, float& y)
{
	y = atan2(R(1,0),R(0,0));
	p = atan2(-R(2,0), sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
	r = atan2(R(2,1),R(2,2));

}





