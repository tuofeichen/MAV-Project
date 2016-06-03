/**
 * @file AnalyticBasedTME.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the AnalyticBasedTME class.
 *
 */

#ifndef SRC_ANALYTICBASEDTME_H_
#define SRC_ANALYTICBASEDTME_H_

#include <vector>

#include "Eigen/Core"

/**
 * @class AnalyticBasedTME AnalyticBasedTME.h "AnalyticBasedTME.h"
 * @brief The AnalyticBasedTME class estimates the transformation matrix parameter in lest-square sense
 */
class AnalyticBasedTME
{
public:
	/**
	 * @brief estimate estimates the transformation matrix parameter
	 * @note This function must be thread safe!!!
	 *
	 * @param keys3D1 3D keypoints of image 1 (input)
	 * @param matchIdx1 matched keypoint index of image 1 (input)
	 * @param keys3D2 3D keypoints of image 2 (input)
	 * @param matchIdx2 matched keypoint index of image 2 (input)
	 * @param activeIdx active index of the matches (input)
	 * @param transformMat transformation matrix to transform the input to the target (output)
	 *
	 * @return Returns false if estimation is invalid, true otherwise.
	 */
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
