/**
 * @file Frame.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the Frame class.
 *
 */
#include "Frame.h"

#define NEIGHBOR_X 3
#define NEIGHBOR_Y 10

namespace SLAM
{

  Frame::Frame():
  id(new int), keyFrameFlag(new bool), newNodeFlag(new bool), dummyFrameFlag(new bool), badFrameFlag(new int), pos(new Eigen::Matrix4f),
  time(new double), rgb(new cv::Mat), gray(new cv::Mat), depth(new cv::Mat),\
  keypoints(new std::vector<cv::KeyPoint>()),  keypoints3D(new std::vector<Eigen::Vector3f>()),\
  descriptors(new cv::Mat),averageFeatureDescriptor(new cv::Mat)
  {
    std::cout << "blank constructor " << std::endl;
  }

Frame::Frame(boost::shared_ptr<cv::Mat>& rgbImage, boost::shared_ptr<cv::Mat>& grayImage, boost::shared_ptr<cv::Mat>& depthImage, boost::shared_ptr<double>& timeStamp)
: id(new int), keyFrameFlag(new bool), newNodeFlag(new bool), dummyFrameFlag(new bool), badFrameFlag(new int), pos(new Eigen::Matrix4f),time(timeStamp),
  rgb(rgbImage), gray(grayImage), depth(depthImage),
  keypoints(new std::vector<cv::KeyPoint>()),  keypoints3D(new std::vector<Eigen::Vector3f>()),
   descriptors(new cv::Mat),averageFeatureDescriptor(new cv::Mat)
{
	*id = -1;
	*newNodeFlag = false;
	*keyFrameFlag = false;
	*dummyFrameFlag = false;
	*badFrameFlag = 0;
	*pos = Eigen::Matrix<float, 4, 4>::Identity();
}

Frame& Frame::operator=(const Frame& other)
{
      int refc = 0;
    // should be fixed in the code later
      refc = averageFeatureDescriptor -> u ? (averageFeatureDescriptor->u->refcount) : 0;

        // std::cout << "shared_ptr count "<< rgb.use_count()<< std::endl;
      if (refc == averageFeatureDescriptor.use_count()) // need error handling
      {
        if (refc==1)
          averageFeatureDescriptor->deallocate();
      }

      rgb    = other.rgb;
      gray   = other.gray;
      depth  = other.depth;
      time   = other.time;
      id     = other.id;
      newNodeFlag      = other.newNodeFlag;
      badFrameFlag     = other.badFrameFlag;
      keyFrameFlag     = other.keyFrameFlag;
      dummyFrameFlag   = other.dummyFrameFlag;
      keypoints        = other.keypoints;
      keypoints3D      = other.keypoints3D;
      descriptors      = other.descriptors;
      averageFeatureDescriptor = other.averageFeatureDescriptor;
      // std::cout << "mat count " << refc << std::endl;

      return *this;
}


void Frame::setKeypoints(boost::shared_ptr<std::vector<cv::KeyPoint>> keys)
{
	// TODO check if its faster to copy it or remove the points
	keypoints->clear();
	keypoints3D->clear();

	// remove keypoints with no depth value
	for(std::vector<cv::KeyPoint>::iterator i = keys->begin(); i != keys->end(); ++i)
	{
		const int col = static_cast<int>(i->pt.x);
		const int row = static_cast<int>(i->pt.y);
		uint16_t depthVal = depth->at<uint16_t>(row, col);
		long depthSum = 0;

		if(depthVal != 0 && depthVal <= static_cast<uint16_t>(3.5*depthScale))
		// if (depthVal <= static_cast<uint16_t>(3.5*depthScale))
		{

			for (int nx = -NEIGHBOR_X/2; nx < NEIGHBOR_X/2; nx ++)
			{
				for (int ny = -NEIGHBOR_Y/2; ny < NEIGHBOR_Y/2; ny ++)
				{
					if ((row+nx)>0 && (row+nx)<240 && (col+ny) > 0 && (col+ny)<320)
						depthSum += (depth->at<uint16_t>(row+nx,col+ny));// - depthVal)*(depth->at<uint16_t>(row+nx,col+ny) - depthVal);
				}
			}

			uint16_t depthAvg = depthSum / (NEIGHBOR_X * NEIGHBOR_Y);
			depthSum = 0;
			for (int nx = -NEIGHBOR_X/2; nx < NEIGHBOR_X/2; nx ++)
			{
				for (int ny = -NEIGHBOR_Y/2; ny < NEIGHBOR_Y/2; ny ++)
				{
					depthSum += (depthAvg - depth->at<uint16_t>(row+nx,col+ny))*(depthAvg - depth->at<uint16_t>(row+nx,col+ny));
				}
			}


			if ((abs(static_cast<uint16_t>(depthSum / (NEIGHBOR_X*NEIGHBOR_Y))) >= 0))
			{
			// std::cout << "keypoint threshold is " << (depthSum / (NEIGHBOR*NEIGHBOR)) << std::endl;
			// std::cout << "  keypoint depth is " << depthVal << std::endl;

			keypoints->push_back(*i);

			Eigen::Vector3f tmp;
			tmp.z() = static_cast<float>(depthVal) * idepthScale;
			tmp.x() = tmp.z() * (static_cast<float>(col) - cx) * ifx; // (scaling)
			tmp.y() = tmp.z() * (static_cast<float>(row) - cy) * ify;
			keypoints3D->push_back(tmp);
			}
		}
	}
	assert(keypoints->size() == keypoints3D->size());
}

// void setDescriptors(boost::shared_ptr<cv::Mat> descs)
// {

//   *descriptors = *descs;
//   setAverageDescriptors();
// }



void Frame::setAverageDescriptors()
{
	assert(descriptors);
	int type = descriptors->type();
	boost::shared_ptr<cv::Mat> averageMat(new cv::Mat(1, descriptors->cols, type));
	for (int col = 0; col < descriptors->cols; ++col)
	{
//		std::cout << "Descriptors: " << std::endl;
//		std::cout << descriptors->col(col) << std::endl;
//		std::cout << "Sum: " << cv::sum(descriptors->col(col)) << std::endl;
//		for (int row = 0; row < descriptors->rows; ++row)
//		{
//			averageMat->at<float>(0,col) += descriptors->at<float>(row,col);
//
//			std::cout << "AverageDescriptors: " << std::endl;
//			std::cout << averageMat->at<float>(0,col) << std::endl;
//		}
		averageMat->at<float>(0,col) = static_cast<float>(cv::sum(descriptors->col(col)).val[0]);
//		std::cout << "AverageDescriptors: " << std::endl;
//		std::cout << averageMat->at<float>(0,col) << std::endl;
		averageMat->at<float>(0,col) = averageMat->at<float>(0,col) / static_cast<float>(descriptors->rows);
	}
	averageFeatureDescriptor = averageMat;
}

}
