 /**
 * @file AsusProLive.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the AsusProLive class using OpenNI and OpenCV
 *
 */

#ifndef ASUSPROLIVE_H_
#define ASUSPROLIVE_H_

#include <vector>

#include "Eigen/Core"
#include "opencv2/videoio/videoio.hpp"

#include "IRGBDSensor.h"

namespace HAL {

/**
 * @class AsusProLive AsusProLive.h "AsusProLive.h"
 * @brief The AsusProLive class is the OpenNI driver for the Asus Xtion Pro Live
 */
class AsusProLive : public IRGBDSensor
{
public:
	/**
	 * @brief Constructor
	 */
	AsusProLive();

	/**
	 * @brief Destructor
	 */
	virtual ~AsusProLive();

	//
	// see IRGBDSensor for more infos
	virtual bool start();
	virtual void stop();
	virtual bool grab(boost::shared_ptr<cv::Mat>& rgbImage, boost::shared_ptr<cv::Mat>& grayImage, boost::shared_ptr<cv::Mat>& depthImage, boost::shared_ptr<double>& timeStamp);
	virtual bool grab(boost::shared_ptr<cv::Mat>& rgbImage, boost::shared_ptr<cv::Mat>& grayImage, boost::shared_ptr<cv::Mat>& depthImage, boost::shared_ptr<cv::Mat>& ptCloud, boost::shared_ptr<double>& timeStamp);

	/**
	 * @brief checks if the driver is still running
	 *
	 * @return true if the driver is running, false otherwise
	 */
	volatile bool running() { return true;}

private:
	cv::VideoCapture asusCapture;
};

} /* namespace HAL */

#endif /* ASUSPROLIVE_H_ */
