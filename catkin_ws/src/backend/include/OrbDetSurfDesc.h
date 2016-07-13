 /**
 * @file OrbDetSurfDesc.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the OrbDetSurfDesc class
 *
 */
#ifndef ORBDETSURFDESC_H_
#define ORBDETSURFDESC_H_

#include "opencv2/xfeatures2d.hpp"


#include "IFeatures.h"

namespace SLAM {

/**
 * @class OrbDetSurfDesc OrbDetSurfDesc.h "OrbDetSurfDesc.h"
 * @brief The OrbDetSurfDesc class detects feature with the ORB detector and extract the descriptions with SURF
 */
class OrbDetSurfDesc : public IFeatures {
public:
	/**
	 * @brief Constructor
	 *
	 * @param aRatio ratio for the ratio test(in)
	 * @param nrOfKeyPoints maximal number of key points to return(in)
	 * @param minMatches minimal number of matches (in)
	 * @param sufficientMatches sufficient matches (in)
	 */
	OrbDetSurfDesc(float aRatio, int maxNrOfFeatures, int minMatches, int sufficientMatches);

	/**
	 * @brief Destructor
	 */
	virtual ~OrbDetSurfDesc();

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
	cv::Ptr<cv::ORB> detecter;
	cv::Ptr<cv::xfeatures2d::SURF> extracter;
};

} /* namespace SLAM */

#endif /* ORBDETSURFDESC_H_ */
