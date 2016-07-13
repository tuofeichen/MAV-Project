/**
 * @file OrbDetSurfDesc.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the OrbDetSurfDesc class.
 *
 */

#include "opencv2/imgproc/imgproc.hpp"

#include "OrbDetSurfDesc.h"


namespace SLAM {

OrbDetSurfDesc::OrbDetSurfDesc(float aRatio, int maxNrOfFeatures, int minMatches, int sufficientMatches)
: ratio(aRatio), sufficientNrOfMatches(sufficientMatches), minNrOfMatches(minMatches),
  detecter(cv::ORB::create(maxNrOfFeatures)),
  extracter(cv::xfeatures2d::SURF::create()) // is thread safe
{ }

OrbDetSurfDesc::~OrbDetSurfDesc()
{ }

boost::shared_ptr<std::vector<cv::KeyPoint>> OrbDetSurfDesc::detect(const cv::Mat& img)
{
	boost::shared_ptr<std::vector<cv::KeyPoint>> keypoints(new std::vector<cv::KeyPoint>);
	detecter->detect(img,*keypoints);
	return keypoints;
}

boost::shared_ptr<cv::Mat> OrbDetSurfDesc::extract(const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts)
{
	boost::shared_ptr<cv::Mat> descriptors(new cv::Mat);
	extracter->compute(img,const_cast<std::vector<cv::KeyPoint>&>(kpts), *descriptors);
	return descriptors;
}

bool OrbDetSurfDesc::match( const std::vector<cv::KeyPoint>& kpts1, const cv::Mat& descs1,
			const std::vector<cv::KeyPoint>& kpts2, const cv::Mat& descs2,
			std::vector<int>& matchesIdx1, std::vector<int>& matchesIdx2) const
{
	cv::flann::Index tree(descs1, cv::flann::KDTreeIndexParams(4), cvflann::FLANN_DIST_EUCLIDEAN);
	cv::Mat indices, dists;
	tree.knnSearch(descs2, indices, dists, 2, cv::flann::SearchParams(16));

	matchesIdx1.clear();
	matchesIdx2.clear();

	for (int i = 0; i < indices.rows && static_cast<int>(matchesIdx1.size()) < sufficientNrOfMatches; ++i)
	{
		float tmpRatio = dists.at<float>(i,0) / dists.at<float>(i,1);
		if (tmpRatio <= ratio)
		{
			matchesIdx1.push_back(indices.at<int>(i,0));
			matchesIdx2.push_back(i);
		}
	}

	if (static_cast<int>(matchesIdx1.size()) < minNrOfMatches)
		return false;
	else
		return true;
}


} /* namespace SLAM */
