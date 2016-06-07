/*
 * AnalyticBasedTME.h
 *
 *  Created on: Apr 10, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#ifndef SRC_ANALYTICBASEDTME_H_
#define SRC_ANALYTICBASEDTME_H_

#include <vector>

#include "Eigen/Core"

class AnalyticBasedTME
{
public:
	static void estimate( // return false if estimation failed and else ture.
				const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
				const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
				const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
				const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
				const std::vector<int>& activeIdx, // in: active index of the matches
				Eigen::Matrix4f& transformMat // out: transformation matrix to transform the input to the target
			); // Note: This function must be thread safe!!!

private:
	AnalyticBasedTME() { } // no objects
};

#endif /* SRC_ANALYTICBASEDTME_H_ */
