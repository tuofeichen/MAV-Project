 /**
 * @file IRGBDSensor.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the IRGBDSensor interface
 *
 */
#ifndef INCLUDES_IRGBDSENSOR_H_
#define INCLUDES_IRGBDSENSOR_H_

#include <boost/smart_ptr/shared_ptr.hpp>

#include "opencv2/core/core.hpp"

namespace HAL {

/**
 * @class IRGBDSensor IRGBDSensor.h "IRGBDSensor.h"
 * @brief The IRGBDSensor interface for RGB-D sensor classes
 */
class IRGBDSensor
{
public:
	/**
	 * @brief Destructor
	 */
	virtual ~IRGBDSensor() { };

	/**
	 * @brief grab grabs images from the RGB-D camera
	 *
	 * @param rgbImage RGB image (output)
	 * @param grayImage gray image (output)
	 * @param depthImage depth image (output)
	 * @param timeStamp time when the images were taken (output)
	 *
	 * @return Returns false if the grabbing procedure failed, true otherwise.
	 */
	virtual bool grab(boost::shared_ptr<cv::Mat>& rgbImage, boost::shared_ptr<cv::Mat>& grayImage, boost::shared_ptr<cv::Mat>& depthImage, boost::shared_ptr<double>& timeStamp) = 0;

	/**
	 * @brief start starts the RGB-D camera
	 *
	 * @return Returns true if the startup was successful and false otherwise.
	 */
	virtual bool start() = 0;

	/**
	 * @brief stop stops the RGB-D camera
	 */
	virtual void stop() = 0;
};

} /* namespace HAL */

#endif /* INCLUDES_IRGBDSENSOR_H_ */
