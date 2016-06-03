 /**
 * @file TestAsusLiveProClass.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file can display the grabed rgb-d images and one can save them
 *
 */


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

//#include "AsusProLive.h"
#include "AsusProLiveOpenNI2.h"

#include <iostream>
#include <vector>

static constexpr bool showFlag = true;
static constexpr bool saveRgbFlag = true;
static constexpr bool saveDepthFlag = true;

void saveRgb(const cv::Mat& rgbIm, const double& t, std::vector<int>& param);
void saveDepth(const cv::Mat& depthIm, const double& t, std::vector<int>& param);

int main()
{

//	HAL::AsusProLive cam;
	HAL::AsusProLiveOpenNI2 cam;
	std::vector<int> compression_params;

	boost::shared_ptr<cv::Mat> bgrImage;
	boost::shared_ptr<cv::Mat> grayImage;
	boost::shared_ptr<cv::Mat> depthImage;
	boost::shared_ptr<double> timeStamp;

	cam.start();

	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);

	sleep(2);

	while(cam.running())
	{
		bool sucRead = false;

		sucRead = cam.grab(bgrImage, grayImage, depthImage, timeStamp);

		if (sucRead)
		{
			if(showFlag)
			{
				const float scaleFactor = 0.05f;
				cv::Mat depthMap;
				depthImage->convertTo( depthMap, CV_8UC1, scaleFactor );
				imshow( "depth map", depthMap );
				imshow( "gray image", *grayImage );
				imshow( "rgb image", *bgrImage );

				if( cv::waitKey( 1 ) >= 0 )
					break;
			}

			if (saveRgbFlag)
			{
				saveRgb(*bgrImage, *timeStamp, compression_params);
			}

			if (saveDepthFlag)
			{
				saveDepth(*depthImage, *timeStamp, compression_params);
			}

		}
	}

	cam.stop();

    return 0;
}


void saveRgb(const cv::Mat& rgbIm, const double& t, std::vector<int>& param)
{
	const std::string path = "/home/user/Downloads/out/rgb/";
	const std::string type = ".png";
	std::string imfile = path+std::to_string(t)+type; // TODO uses the times stamp of the rgb image, change with own time stamp

	if(!cv::imwrite(imfile.data(), rgbIm, param))
		std::cerr << "Rgb image could not be saved!" << std::endl;
}

void saveDepth(const cv::Mat& depthIm, const double& t, std::vector<int>& param)
{
	const std::string path = "/home/user/Downloads/out/depth/";
	const std::string type = ".png";
	std::string imfile = path+std::to_string(t)+type; // TODO uses the times stamp of the rgb image, change with own time stamp

	if (!cv::imwrite(imfile, depthIm, param))
		std::cerr << "Depth image could not be saved!" << std::endl;
}



