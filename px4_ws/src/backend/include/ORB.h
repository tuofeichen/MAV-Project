 /**
 * @file ORB.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the ORB class
 *
 */
#ifndef ORB_H_
#define ORB_H_

#include <string>

#include "IFeatures.h"

namespace SLAM {

/**
 * @class ORB ORB.h "ORB.h"
 * @brief The ORB feature detection, extraction and matching class
 */
class ORB : public IFeatures {
public:
	/**
	 * @brief Constructor
	 *
	 * @param aRatio ratio for the ratio test(in)
	 * @param nrOfKeyPoints maximal number of key points to return(in)
	 * @param minMatches minimal number of matches (in)
	 * @param sufficientMatches sufficient matches (in)
	 */
	ORB(float ratio, int nrOfKeyPoints, int minMatches, int sufficientMatches);

	/**
	 * @brief Destructor
	 */
	virtual ~ORB();

	//
	// see IFeatures.h for description
	virtual boost::shared_ptr<std::vector<cv::KeyPoint>> detect(const cv::Mat& img);
	virtual boost::shared_ptr<cv::Mat> extract(const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts);
	virtual bool match(
			const std::vector<cv::KeyPoint>& kpts1, const cv::Mat& descs1,
			const std::vector<cv::KeyPoint>& kpts2, const cv::Mat& descs2,
			std::vector<int>& matchesIdx1, std::vector<int>& matchesIdx2) const; // must be thread safe

private:
	const float ratio;
	const int sufficientNrOfMatches;
	const int minNrOfMatches;
	cv::Ptr<cv::ORB> detecterExtracter;
};

} /* namespace SLAM */

#endif /* ORB_H_ */
