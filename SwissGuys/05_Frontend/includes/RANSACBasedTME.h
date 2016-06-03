 /**
 * @file RANSACBasedTME.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the RANSACBasedTME class
 *
 */
 
#ifndef RANSACBASEDTME_H_
#define RANSACBASEDTME_H_

#include <vector>

#include <Eigen/Core>

#include "Frame.h"

/**
 * @class RANSACBasedTME RANSACBasedTME.h "RANSACBasedTME.h"
 * @brief The RANSACBasedTME class estimates the transformation and the information matrix with the RANSAC algorithm
 */
class RANSACBasedTME
{
public:
	// settings
	static constexpr int iter = Frame::ransacMaxIter;
	static constexpr float thresholdAbsDistTest = Frame::ransacThresholdAbsolutDistTest;
	static constexpr float maxDistInlier = Frame::ransacMaxDistInlier;
	static constexpr int minInlier = Frame::ransacMinInlier;
	static constexpr float termInlierPct = Frame::ransacTermInlierPct;

	/**
	 * @brief estimateTrafo estimates the transformation matrix
	 * @note This function must be thread safe!!!
	 *
	 * @param keys3D1 3D keypoints of image 1 (input)
	 * @param matchIdx1 matched keypoint index of image 1 (input)
	 * @param keys3D2 3D keypoints of image 2 (input)
	 * @param matchIdx2 matched keypoint index of image 2 (input)
	 * @param transformMat transformation matrix to transform the input to the target (output)
	 * @param informationMat information matrix of the transformation (output)
	 *
	 * @return Returns false if estimation failed, true otherwise.
	 */
	static bool estimateTrafo( // return false if estimation failed and else ture.
			const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
			const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
			const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
			const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
			Eigen::Matrix4f& transformMat, // out: transformation matrix to transform the input to the target
			Eigen::Matrix<float, 6, 6>& informationMat // out: information matrix of the transformation
			); // Note: This function must be thread safe!!!

private:
	RANSACBasedTME() { } // no objects

	static bool estimateTransformationMatrix( // return true if transformation matrix is valid and false if invalid
			const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
			const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
			const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
			const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
			const std::vector<int>& consensus, // in: index of the matches in the consensus
			Eigen::Matrix4f& transformMat // out: transformation matrix to transform the input to the target
			);
	static int computeSquaredDistance(
			const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
			const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
			const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
			const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
			const Eigen::Matrix4f& transformMat,
			const float maxSquaredDist,
			std::vector<int>& consensus, // out: index of the matches in the new consensus
			float& mse
			);
	static bool checkReliability(const Eigen::Matrix4f& transformMat, bool& isSmall);

	static constexpr float squaredMaxDistInlier = maxDistInlier * maxDistInlier; // m²
};

#endif /* RANSACBASEDTME_H_ */
