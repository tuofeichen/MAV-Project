/*
 * VisualOdometry.cpp
 *
 *  Created on: May 15, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#include "FeatureDEM.h"
#include "RANSACBasedTME.h"

#include "VisualOdometry.h"

bool VisualOdometry::setKeypoints(Frame& frame)
{
	bool ret = false;
	std::vector<cv::KeyPoint> keypoints(Frame::maxNrOfKeyPoints);
	
	FeatureDEM::detect(frame.getGray(), keypoints);

	frame.setKeypoints(keypoints);
	
	if(frame.getKeypoints().size() >= minNrOfKeyPoints)
	{
		FeatureDEM::extract(frame.getGray(), frame.getKeypoints(), frame.setDescriptors());
		ret = true;
	}

	// number of 3D key points
	return ret;
}

VisualOdometry::Result VisualOdometry::checkReliability(const Eigen::Matrix4f& tm, const double& dt)
{
	// TODO use atan2 out of a table
	float roll = atan2(tm(2,1),tm(2,2)); // roll (around x)
	float pitch = atan2(-tm(2,0),sqrt(tm(2,1)*tm(2,1)+tm(2,2)*tm(2,2))); // pitch (around y)
	float yaw = atan2(tm(1,0),tm(0,0)); // yaw (around z)

	float maxAngle = std::max(fabs(roll),std::max(fabs(pitch),fabs(yaw))); //TODO abs() needed?
	float distSqrt = (tm.topRightCorner<3,1>()).squaredNorm();

	if((distSqrt/dt) > maxSquaredTranslationPerSecond || (maxAngle/dt) > maxAngularVelocit)
		return invalid;

	if(distSqrt > minSquaredTranslation || maxAngle > minRotation) // movment big enough for a new node
		return valid;
	else
		return small;

}

std::vector<int> VisualOdometry::estimateTrafo(const Frame& srcFrame, const Frame& targetFrame,
		Eigen::Matrix4f& tmSrcToTarget, Eigen::Matrix<float, 6, 6>& imEstSrcToTarget, bool& valid)
{
	valid = false;

	//
	// feature detecting, extracting and matching
	//
	std::vector<int> srcMatches(Frame::maxNrOfKeyPoints);
	std::vector<int> targetMatches(Frame::maxNrOfKeyPoints);
	

	FeatureDEM::match(srcFrame.getKeypoints(), srcFrame.getDescriptors(), targetFrame.getKeypoints(), targetFrame.getDescriptors(), srcMatches, targetMatches);


	if (srcMatches.size() >= minNrOfMatches)
	{
		//
		// estimate transformation matrix
		//
		valid = RANSACBasedTME::estimateTrafo(srcFrame.getKeypoints3D(), srcMatches, targetFrame.getKeypoints3D(), targetMatches, tmSrcToTarget, imEstSrcToTarget);
	}

	return srcMatches;
}
