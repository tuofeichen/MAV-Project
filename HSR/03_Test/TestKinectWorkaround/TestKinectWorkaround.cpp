#include <iostream>

#include <boost/thread.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp" //debug only

#include "KinectV2DriverWorkaround.h"

using namespace HAL;
using namespace std;


static void testCallback(void* context, cv::Mat& rgbImg, cv::Mat& depthImg, double& timeStamp)
{
//	cv::namedWindow("DepthImage");
//	cv::imshow("DepthImage", depthImg);

	cv::namedWindow("RgbImage");
	cv::imshow("RgbImage", rgbImg);

	((KinectV2DriverWorkaround*)context)->imagesProcessed();

	cv::waitKey(1);
}

int main()
{
	KinectV2DriverWorkaround kinect;
	kinect.registerCallback(testCallback,&kinect);
	kinect.setSaveFlag(true); // save images in ~/Downloads/out/ (sub folders rgb and depth)
	boost::thread handler = boost::thread(&KinectV2DriverWorkaround::start, &kinect);
	handler.join();
	return 0;
}
