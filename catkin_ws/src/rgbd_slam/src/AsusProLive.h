/*
 * AsusProLive.h
 *
 *  Created on: May 13, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#ifndef ASUSPROLIVE_H_
#define ASUSPROLIVE_H_

#include "opencv2/videoio.hpp"

#include "Frame.h"

class AsusProLive
{
public:
	static bool start();
	static void stop();
	static bool grab(Frame& frame);

private:
	AsusProLive() { } // no objects
	static cv::VideoCapture asusCapture;

	static cv::Mat dImg;
	static cv::Mat cImg;
	static cv::Mat gImg;
};

#endif /* ASUSPROLIVE_H_ */
