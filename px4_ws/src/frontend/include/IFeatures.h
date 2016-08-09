/**
 * @file IFeatures.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the IFeatures interface.
 *
 */
#ifndef IFEATURES_H_
#define IFEATURES_H_

#include <boost/shared_ptr.hpp>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"

namespace SLAM {

/**
 * @class IFeatures IFeatures.h "IFeatures.h"
 * @brief The IFeatures interface for a feature class.
 */
class IFeatures {
public:
	/**
	 * @brief Destructur
	 */
	virtual ~IFeatures() { }

	/**
	 * @brief detects feature points in the given image
	 *
	 * @param img image (input)
	 *
	 * @return features points
	 */
	virtual boost::shared_ptr<std::vector<cv::KeyPoint>> detect(const cv::Mat& img) = 0;

	/**
	 * @brief extracts feature descriptor vectors in the given image
	 *
	 * @param img image (input)
	 * @param kpts feature points (input)
	 *
	 * @return features descriptors
	 */
	virtual boost::shared_ptr<cv::Mat> extract(const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts) = 0;

	/**
	 * @brief matches two feature descriptor vectors and returns the index of the matched features.
	 * 		  The feature point with index 1 in matchesIdx1 matches with the feature point with index 1 in matchesIdx2.
	 *
	 * @param kpts1 feature points of image 1 (input)
	 * @param descs1 feature description vectors of image 1 (input)
	 * @param kpts2 feature points of image 2 (input)
	 * @param descs2 feature description vectors of image 2 (input)
	 * @param matchesIdx1 index of the matched feature points in image 1 (out)
	 * @param matchesIdx2 index of the matches feature points in image 2 (out)
	 *
	 * @return true if enough matches where found, else false
	 */
	// virtual bool match(
	// 		const std::vector<cv::KeyPoint>& kpts1, const cv::Mat& descs1,
	// 		const std::vector<cv::KeyPoint>& kpts2, const cv::Mat& descs2,
	// 		std::vector<int>& matchesIdx1, std::vector<int>& matchesIdx2) const = 0; // must be thread safe

	 virtual bool match( const std::vector<cv::KeyPoint>& kpts1, const cv::Mat& descs1,
			const std::vector<cv::KeyPoint>& kpts2, const cv::Mat& descs2,
			std::vector<cv::DMatch>& matches) const = 0;

};

} /* namespace SLAM */

#endif /* IFEATURES_H_ */
