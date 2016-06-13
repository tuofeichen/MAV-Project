/*
 * Frame.cpp
 *
 *  Created on: May 13, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#include "Frame.h"

void Frame::setKeypoints(std::vector<cv::KeyPoint>& keys)
{
	keypoints.clear();
	keypoints3D.clear();

	// remove keypoints with no depth value
	for(std::vector<cv::KeyPoint>::iterator i = keys.begin(); i != keys.end(); ++i)
	{
		const int col = static_cast<int>(i->pt.x);
		const int row = static_cast<int>(i->pt.y);
		uint16_t depthVal = depth.at<uint16_t>(row, col);
		if(depthVal != 0)
		{
			keypoints.push_back(*i);

			Eigen::Vector3f tmp;
			tmp.z() = static_cast<float>(depthVal) * idepthScale;
			tmp.x() = tmp.z() * (static_cast<float>(col) - cx) * ifx;
			tmp.y() = tmp.z() * (static_cast<float>(row) - cy) * ify;
			keypoints3D.push_back(tmp);
		}
	}
}
