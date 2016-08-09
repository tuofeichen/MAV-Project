 /**
 * @file RANSACBasedTME.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the RANSACBasedTME class
 *
 */
#ifndef RANSACBASEDTME_H_
#define RANSACBASEDTME_H_

///< enables / disables the mahalanobis distance
//#define WITH_MAHALANOBIS

#include "ITransformMatEst.h"

namespace SLAM {

/**
 * @class RANSACBasedTME RANSACBasedTME.h "RANSACBasedTME.h"
 * @brief The RANSACBasedTME class implements the RANSAC algorithm to estimate the transformation parameters
 */
class RANSACBasedTME : public ITransformMatEst {
public:
	/**
	 * @brief Constructor
	 *
	 * @param iterations max iterations (in)
	 * @param maxDistForInlier max distance for inlier (in)
	 * @param thresholdForAbsDistTest threshold for the absolute distanece test (in)
	 * @param terminationInlierPct percentage of inlier to terminate the algorithm (in)
	 * @param inlierMin minimal number of inlier (in)
	 */
	RANSACBasedTME(int iterations, float maxDistForInlier, float thresholdForAbsDistTest, float terminationInlierPct = 0.6f, int inlierMin = 50);

	/**
	 * @brief Destructor
	 */
	virtual ~RANSACBasedTME();

	//
	// see IPoseGraph.h for description
	virtual bool estimateTrafo( // return false if estimation failed and else ture.
				const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
				const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
				const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
				const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
				Eigen::Matrix4f& transformMat, // out: transformation matrix to transform the input to the target
				Eigen::Matrix<double, 6, 6>& informationMat, // out: information matrix of the transformation
				std::vector<int>& consensus
				) const; // Note: This function must be thread safe!!!

private:
	bool estimateTransformationMatrix( // return true if transformation matrix is valid and false if invalid
			const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
			const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
			const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
			const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
			const std::vector<int>& consensus, // in: index of the matches in the consensus
			Eigen::Matrix4f& transformMat // out: transformation matrix to transform the input to the target
			) const;
	int computeSquaredDistance(
			const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
			const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
			const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
			const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
			const Eigen::Matrix4f& transformMat,
			const float maxSquaredDist,
			std::vector<int>& consensus, // out: index of the matches in the new consensus
			float& mse
			) const;

#ifdef WITH_MAHALANOBIS

	inline double depthStdDev(double depth) const
	{
	  return depthCovZFactor * depth * depth;
	}
	inline double depthCovariance(double depth) const
	{
	  double stddev = depthStdDev(depth);
	  return stddev * stddev;
	}
	double mahalanobisDistance(const Eigen::Vector4f& x1,
	                      const Eigen::Vector4f& x2,
	                      const Eigen::Matrix4f& tf_1_to_2) const;

	//
	// Kinect V1
	static constexpr double depthCovZFactor = 1.425e-3; // Source: RGDB-SLAM ROS Packet for Kinect V1
	static constexpr double camAngleX = 58.0/180.0*M_PI;
	static constexpr double camAngleY = 45.0/180.0*M_PI;
	static constexpr double camResolX = 640;
	static constexpr double camResolY = 480;
	static const double rasterStddevX;
	static const double rasterStddevY;
	static const double rasterCovX;
	static const double rasterCovY;

	//
	// Kinect V2
//	const double depthCovZFactor = 1.425e-3; //TODO
//	const double camAngleX = 66.6/180.0*M_PI;
//	const double camAngleY = 52.5/180.0*M_PI;
//	const double camResolX = 480;
//	const double camResolY = 360;

#endif

	const int iter;
	const float thresholdAbsolutDistanceTest;
	const float squaredMaxDistInlier; // in m or in standard deviations if mahalanobis distance is used
	const int minInlier;
	const float termInlierPct;
};

} /* namespace SLAM */

#endif /* RANSACBASEDTME_H_ */
