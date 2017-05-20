/*
 * AsusProLive.cpp
 *
 *  Created on: May 13, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#include <ctime>
#include <iostream>

#include "AsusProLive.h"

cv::VideoCapture AsusProLive::asusCapture;
cv::Mat AsusProLive::dImg(Frame::rows, Frame::cols, CV_16UC1);
cv::Mat AsusProLive::cImg(Frame::rows, Frame::cols, CV_8UC3);
cv::Mat AsusProLive::gImg(Frame::rows, Frame::cols, CV_8UC1);

bool AsusProLive::start()
{
	asusCapture.open( cv::CAP_OPENNI2_ASUS );
	if( !asusCapture.isOpened() )
	{
		asusCapture.open( cv::CAP_OPENNI2_ASUS );
	}

	if( !asusCapture.isOpened() )
		return false;

	//
	// color settings
	bool modeRes = asusCapture.set( cv::CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, cv::CAP_OPENNI_VGA_30HZ);

	std::cout << "Image Generator == " << asusCapture.get(cv::CAP_OPENNI_IMAGE_GENERATOR+cv::CAP_PROP_OPENNI_GENERATOR_PRESENT) << std::endl;
	std::cout << "Image FPS ==" << asusCapture.get( cv::CAP_OPENNI_IMAGE_GENERATOR+cv::CAP_PROP_FPS ) << std::endl;

	//
	// depth settings
	modeRes &= asusCapture.set( cv::CAP_OPENNI_DEPTH_GENERATOR + cv::CAP_PROP_OPENNI_REGISTRATION, false );

	std::cout << "Depth Generator == " << asusCapture.get(cv::CAP_OPENNI_DEPTH_GENERATOR+cv::CAP_PROP_OPENNI_GENERATOR_PRESENT) << std::endl;
	std::cout << "Depth FPS ==" << asusCapture.get( cv::CAP_OPENNI_DEPTH_GENERATOR+cv::CAP_PROP_FPS ) << std::endl;
	std::cout << "Registration == " << asusCapture.get(cv::CAP_OPENNI_DEPTH_GENERATOR+cv::CAP_PROP_OPENNI_REGISTRATION) << std::endl;
	std::cout << "focal length in pixel == " << asusCapture.get(cv::CAP_OPENNI_DEPTH_GENERATOR+cv::CAP_PROP_OPENNI_FOCAL_LENGTH) << std::endl;
	std::cout << "frame max depth in mm == " << asusCapture.get(cv::CAP_OPENNI_DEPTH_GENERATOR+cv::CAP_PROP_OPENNI_FRAME_MAX_DEPTH) << std::endl;
	std::cout << "Baseline in mm == " << asusCapture.get(cv::CAP_OPENNI_DEPTH_GENERATOR+cv::CAP_PROP_OPENNI_BASELINE) << std::endl;

	//
	// general
	modeRes &= asusCapture.set( cv::CAP_PROP_OPENNI2_MIRROR, false );
	modeRes &= asusCapture.set( cv::CAP_PROP_OPENNI2_SYNC, false );

	std::cout << "Mirror == " << asusCapture.get(cv::CAP_PROP_OPENNI2_MIRROR) << std::endl;
	std::cout << "Sync == " << asusCapture.get(cv::CAP_PROP_OPENNI2_SYNC) << std::endl;


	if (!modeRes)
		return false;

	return true;

}

void AsusProLive::stop()
{
	asusCapture.release();
}

bool AsusProLive::grab(Frame& frame)
{
	if (asusCapture.grab())
	{
//		frame.setTime() = asusCapture.get(cv::CAP_PROP_POS_MSEC); //TODO tacke time of camera!
		frame.setTime() = static_cast<double>( clock () ) /  CLOCKS_PER_SEC;

		assert(asusCapture.retrieve( dImg, cv::CAP_OPENNI_DEPTH_MAP));
		assert(asusCapture.retrieve( cImg, cv::CAP_OPENNI_BGR_IMAGE ));
		assert(asusCapture.retrieve( gImg, cv::CAP_OPENNI_GRAY_IMAGE ));

		dImg.copyTo(frame.setDepth());
		cImg.copyTo(frame.setRgb());
		gImg.copyTo(frame.setGray());

		return true;
	}
	else
		return false;
}
