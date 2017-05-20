 /**
 * @file FeatureDEM.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the FeatureDEM class
 *
 */

#ifndef FEATUREDEM_H_
#define FEATUREDEM_H_

#include <vector>

#include <boost/shared_ptr.hpp>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"

#include "Frame.h"

/**
 * @class FeatureDEM FeatureDEM.h "FeatureDEM.h"
 * @brief The FeatureDEM class detects, extracts, and matches feature points
 */
class FeatureDEM
{
public:
	static constexpr int nrOfKeyPoints = Frame::maxNrOfKeyPoints;

	/**
	 * @brief detect detects features in an image
	 *
	 * @param img image where this function has to detect feature points (input)
	 * @param keypoints found feature points (output)
	 */
	static void detect(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints);
	
	/**
	 * @brief extract describes features in an image
	 *
	 * @param img image where the features are (input)
	 * @param kpts found feature points (input)
	 * @param descriptors descriptor of the feature points (output)
	 */
	static void extract(const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts, cv::Mat& descriptors);
	
	/**
	 * @brief match matches two feature descriptors
	 *
	 * @param kpts1 found feature points in image 1 (input)
	 * @param descs1 descriptor of the feature points in image 1 (input)
	 * @param kpts2 found feature points in image 2 (input)
	 * @param descs2 descriptor of the feature points  in image 2 (input)
	 * @param matchesIdx1 match index from image 1 to image 2 (output)	
	 * @param matchesIdx2 match index from image 2 to image 1 (output)	 
	 */
	static void match(
			const std::vector<cv::KeyPoint>& kpts1, const cv::Mat& descs1,
			const std::vector<cv::KeyPoint>& kpts2, const cv::Mat& descs2,
			std::vector<int>& matchesIdx1, std::vector<int>& matchesIdx2); // must be thread safe

private:
	FeatureDEM() { } // no objects

	static cv::Ptr<cv::ORB> detecterExtracter;
	static cv::Ptr<cv::BFMatcher> matcher;
};

#endif /* FEATUREDEM_H_ */
