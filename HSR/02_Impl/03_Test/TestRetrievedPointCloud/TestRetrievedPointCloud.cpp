 /**
 * @file TestRetrievedPointCloud.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file can compares the point cloud given by the Asus with the calculated point cloud
 *
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "pcl/io/pcd_io.h"
//#include "Eigen/Core"

#include "AsusProLive.h"

#include <iostream>
#include <vector>


void getPc(const boost::shared_ptr<cv::Mat>& bgrImage, const boost::shared_ptr<cv::Mat>& depthIm, pcl::PointCloud<pcl::PointXYZRGB>& map);
void saveMap(const std::string& file, pcl::PointCloud<pcl::PointXYZRGB>& map);
void saveMap(const std::string& file, pcl::PointCloud<pcl::PointXYZ>& map);
void convertPc(boost::shared_ptr<cv::Mat>& inPointCloud, pcl::PointCloud<pcl::PointXYZ>& outPointCloud);

int main()
{

	HAL::AsusProLive cam;
	std::vector<int> compression_params;

	boost::shared_ptr<cv::Mat> bgrImage;
	boost::shared_ptr<cv::Mat> grayImage;
	boost::shared_ptr<cv::Mat> depthImage;
	boost::shared_ptr<cv::Mat> pointCloud;
	pcl::PointCloud<pcl::PointXYZRGB> ownMap;
	boost::shared_ptr<double> timeStamp;

	cam.start();

	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);

	cam.grab(bgrImage, grayImage, depthImage, pointCloud, timeStamp);
	cam.grab(bgrImage, grayImage, depthImage, pointCloud, timeStamp);
	cam.grab(bgrImage, grayImage, depthImage, pointCloud, timeStamp);

	bool sucRead = false;

	sucRead = cam.grab(bgrImage, grayImage, depthImage, pointCloud, timeStamp);

	if (sucRead)
	{
		pcl::PointCloud<pcl::PointXYZ> convPtC;

		convertPc(pointCloud, convPtC);

		getPc(bgrImage, depthImage, ownMap);

		saveMap("ownMap.pcd", ownMap);
		std::cout << "Own map is saved" << std::endl;

		saveMap("asusMap.pcd", convPtC);
		std::cout << "Asus map is saved" << std::endl;
	}

    return 0;
}


void getPc(const boost::shared_ptr<cv::Mat>& bgrImage, const boost::shared_ptr<cv::Mat>& depthIm, pcl::PointCloud<pcl::PointXYZRGB>& map)
{
	map.clear();

	float idepthScale = 1 / 1000.0;
	float cx = 319.5;
	float cy = 239.5;
	float ifx = 1 / 570.0;
	float ify = 1 / 570.0;

	float tmpX, tmpY, tmpZ;
	const int width = depthIm->cols;
	const int hight = depthIm->rows;

	for (int row = 0; row < hight; ++row) {

		for (int col = 0; col < width; ++col) {

				uint16_t depth = depthIm->at<uint16_t>(row * width + col);
				if(depth != 0) {
					tmpZ = static_cast<float>(depth)*idepthScale;
					tmpX = tmpZ * (static_cast<float>(col) - cx) * ifx;
					tmpY = tmpZ * (static_cast<float>(row) - cy) * ify;

					const int rgbCol = 3*col;
					pcl::PointXYZRGB tmpPoint(bgrImage->at<uint8_t>(row, rgbCol+2), bgrImage->at<uint8_t>(row, rgbCol+1), bgrImage->at<uint8_t>(row, rgbCol));
					tmpPoint.x = tmpX;
					tmpPoint.y = tmpY;
					tmpPoint.z = tmpZ;
					map.push_back(tmpPoint);
				}
		}
	}
}

void saveMap(const std::string& file, pcl::PointCloud<pcl::PointXYZRGB>& map)
{
	pcl::io::savePCDFile (file, map);
}

void saveMap(const std::string& file, pcl::PointCloud<pcl::PointXYZ>& map)
{
	pcl::io::savePCDFile (file, map);
}

void convertPc(boost::shared_ptr<cv::Mat>& inPointCloud, pcl::PointCloud<pcl::PointXYZ>& outPointCloud)
{
	const int width = inPointCloud->cols;
	const int hight = inPointCloud->rows;
	for (int row = 0; row < hight; ++row) {
		for (int col = 0; col < width; ++col) {

			const int xyzCol = 3 * col;
			pcl::PointXYZ tmpPoint;
			tmpPoint.x = inPointCloud->at<float>(row, xyzCol);
			tmpPoint.y = -inPointCloud->at<float>(row, xyzCol + 1);
			tmpPoint.z = inPointCloud->at<float>(row, xyzCol + 2);
			outPointCloud.push_back(tmpPoint);
		}
	}

}




