/*
 * FeatureDEM.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#include "FeatureDEM.h"

cv::Ptr<cv::ORB> FeatureDEM::detecterExtracter = cv::ORB::create(nrOfKeyPoints, 1.2f, 2, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
cv::Ptr<cv::BFMatcher> FeatureDEM::matcher = new cv::BFMatcher(cv::NORM_HAMMING, true);

void FeatureDEM::detect(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints)
{
	keypoints.clear();
	detecterExtracter->detect(img, keypoints);
}

void FeatureDEM::extract(const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts, cv::Mat& descriptors)
{
	detecterExtracter->compute(img,const_cast<std::vector<cv::KeyPoint>&>(kpts), descriptors);
}

void FeatureDEM::match( const std::vector<cv::KeyPoint>& kpts1, const cv::Mat& descs1,
			const std::vector<cv::KeyPoint>& kpts2, const cv::Mat& descs2,
			std::vector<int>& matchesIdx1, std::vector<int>& matchesIdx2)
{
	std::vector< cv::DMatch > matches;
	matcher->match(descs1,descs2,matches);

	matchesIdx1.clear();
	matchesIdx2.clear();

	for(int i=0; i < matches.size(); ++i)
	{
		matchesIdx1.push_back(matches[i].queryIdx);
		matchesIdx2.push_back(matches[i].trainIdx);
	}
}
