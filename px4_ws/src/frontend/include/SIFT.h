 /**
 * @file SIFT.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the SIFT class
 *
 */
#ifndef SIFT_H_
#define SIFT_H_

#include "opencv2/xfeatures2d.hpp"

#include "IFeatures.h"

namespace SLAM {

/**
 * @class SIFT SIFT.h "SIFT.h"
 * @brief The SIFT feature detection, extraction and matching class
 */
class SIFT : public IFeatures{
public:
	/**
	 * @brief Constructor
	 *
	 * @param aRatio ratio for the ratio test(in)
	 * @param minMatches minimal number of matches (in)
	 * @param sufficientMatches sufficient matches (in)
	 */
	SIFT(float aRatio, int minMatches, int sufficientMatches);

	/**
	 * @brief Destructor
	 */
	virtual ~SIFT();

	//
	// see IFeatures.h for description
	virtual boost::shared_ptr<std::vector<cv::KeyPoint>> detect(const cv::Mat& img);
	virtual boost::shared_ptr<cv::Mat> extract(const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts);
	// virtual bool match( const std::vector<cv::KeyPoint>& kpts1, const cv::Mat& descs1,
	// 					const std::vector<cv::KeyPoint>& kpts2, const cv::Mat& descs2,
	// 					std::vector<int>& matchesIdx1, std::vector<int>& matchesIdx2) const;

	virtual bool match( const std::vector<cv::KeyPoint>& kpts1, const cv::Mat& descs1,
						const std::vector<cv::KeyPoint>& kpts2, const cv::Mat& descs2,
						std::vector<cv::DMatch>& matches) const;



private:
	const float ratio;
	const int sufficientNrOfMatches;
	const int minNrOfMatches;
	cv::Ptr<cv::xfeatures2d::SIFT> detecterExtracter;
};

} /* namespace SLAM */

#endif /* SIFT_H_ */
