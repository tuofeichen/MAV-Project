/**
 * @file ITransformMatEst.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the ITransformMatEst interface.
 *
 */

#ifndef ITRANSFORMMATEST_H_
#define ITRANSFORMMATEST_H_

#include <vector>

#include "Eigen/Core"

namespace SLAM {

/**
 * @class ITransformMatEst ITransformMatEst.h "ITransformMatEst.h"
 * @brief The ITransformMatEst interface for a transformation parameter estimation class.
 */
class ITransformMatEst {
public:
	virtual ~ITransformMatEst() { };

	/**
	 * @brief estimateTrafo estimates the transformation matrix parameter
	 * @note This function must be thread safe!!!
	 *
	 * @param keys3D1 3D keypoints of image 1 (input)
	 * @param matchIdx1 matched keypoint index of image 1 (input)
	 * @param keys3D2 3D keypoints of image 2 (input)
	 * @param matchIdx2 matched keypoint index of image 2 (input)
	 * @param transformMat transformation matrix to transform the input to the target (output)
	 * @param informationMat information matrix of the transformation (output)
	 *
	 * @return Returns false if estimation is invalid, true otherwise.
	 */
	virtual bool estimateTrafo(
			const std::vector<Eigen::Vector3f>& keys3D1,
			const std::vector<int>& matchIdx1,
			const std::vector<Eigen::Vector3f>& keys3D2,
			const std::vector<int>& matchIdx2,
			Eigen::Matrix4f& transformMat,
			Eigen::Matrix<double, 6, 6>& informationMat,
			std::vector<int>& consensus
			) const = 0;
};

} /* namespace SLAM */

#endif /* ITRANSFORMMATEST_H_ */
