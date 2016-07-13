/*
 * FeatureDEM.h
 *
 *  Created on: Mar 10, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#ifndef FEATUREDEM_H_
#define FEATUREDEM_H_

#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"

#include "Frame.h"

class FeatureDEM
{
public:
	static constexpr int nrOfKeyPoints = Frame::maxNrOfKeyPoints;

	static void detect(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints);
	static void extract(const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts, cv::Mat& descriptors);
	static void match(
			const std::vector<cv::KeyPoint>& kpts1, const cv::Mat& descs1,
			const std::vector<cv::KeyPoint>& kpts2, const cv::Mat& descs2,
			std::vector<int>& matchesIdx1, std::vector<int>& matchesIdx2); // must be thread safe

private:
	FeatureDEM() { } // no objects

	static cv::Ptr<cv::ORB> detecterExtracter;
	static cv::Ptr<cv::BFMatcher> matcher; // brute force matcher 
};

#endif /* FEATUREDEM_H_ */
