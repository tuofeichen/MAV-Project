/**
 * @file Frame.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the Frame class.
 *
 */
#include "Frame.h"

namespace SLAM
{

Frame::Frame():
id(new int), keyFrameFlag(new bool),  dummyFrameFlag(new bool), \
time(new double), rgb(new cv::Mat), gray(new cv::Mat), depth(new cv::Mat),\
keypoints(new std::vector<cv::KeyPoint>()),  keypoints3D(new std::vector<Eigen::Vector3f>()),\
descriptors(new cv::Mat),averageFeatureDescriptor(new cv::Mat)
{
  std::cout << "blank constructor " << std::endl;
}

Frame::Frame(boost::shared_ptr<cv::Mat>& rgbImage, boost::shared_ptr<cv::Mat>& grayImage, \
  boost::shared_ptr<cv::Mat>& depthImage, boost::shared_ptr<double>& timeStamp)
: id(new int), keyFrameFlag(new bool),  dummyFrameFlag(new bool), \
time(timeStamp), rgb(rgbImage), gray(grayImage), depth(depthImage),\
 keypoints(new std::vector<cv::KeyPoint>()),  keypoints3D(new std::vector<Eigen::Vector3f>()),\
 descriptors(new cv::Mat),averageFeatureDescriptor(new cv::Mat)
{

	*id = -1;
	*keyFrameFlag = false;
	*dummyFrameFlag = false;

  // int refc = 0;
  //
  // rgbImage.reset();
  //
  // refc = rgb -> u ? (rgb ->u->refcount) : 0;
  // std::cout << "shared_ptr count " << rgb.use_count()<< std::endl;
  // std::cout << "mat count " << refc << std::endl;
  // rgb = gray;
  // rgb.reset(new cv::Mat);

}

Frame::Frame(const Frame& other)
{
    // if (other.getRgb().rows > 2){ // this needs handling b/c of vector set up
    // rgb.reset(new cv::Mat);
    // gray.reset(new cv::Mat);
    // depth.reset(new cv::Mat);
    // *rgb        = other.getRgb();
    // *gray       = other.getGray();
    // *depth      = other.getDepth();
    // }
    // else
    // {
    //   std::cout << "[Warning] frame images deleted " << std::endl;
    // }

    // time.reset(new double);
    // id.reset(new int );
    // keyFrameFlag.reset(new bool);
    // dummyFrameFlag.reset(new bool);
    // keypoints.reset(new std::vector<cv::KeyPoint>);
    // keypoints3D.reset(new std::vector<Eigen::Vector3f>);
    // descriptors.reset(new cv::Mat);
    // // averageFeatureDescriptor->deallocate();
    // averageFeatureDescriptor.reset(new cv::Mat);
    //


    // *time           = other.getTime();
    // *id             = other.getId();
    // *keyFrameFlag   = other.getKeyFrameFlag();
    // *dummyFrameFlag = other.getDummyFrameFlag();
    // *keypoints      = other.getKeypoints();
    // *keypoints3D    = other.getKeypoints3D();
    // *descriptors    = other.getDescriptors();
    //
    rgb    = other.rgb;
    gray   = other.gray;
    depth  = other.depth;
    time   = other.time;
    id     = other.id;
    keyFrameFlag     = other.keyFrameFlag;
    dummyFrameFlag   = other.dummyFrameFlag;
    keypoints        = other.keypoints;
    keypoints3D      = other.keypoints3D;
    descriptors      = other.descriptors;
    averageFeatureDescriptor = other.averageFeatureDescriptor;
}

Frame& Frame::operator=(const Frame& other)
{
      int refc = 0;
      refc = averageFeatureDescriptor -> u ? (averageFeatureDescriptor->u->refcount) : 0;
      // should be fixed in the code later

      // refc = rgb -> u ? (rgb->u->refcount) : 0;
      // std::cout << "shared_ptr count " << averageFeatureDescriptor.use_count()<< std::endl;
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
		if(depthVal != 0)
		{
			keypoints->push_back(*i);

			Eigen::Vector3f tmp;
			tmp.z() = static_cast<float>(depthVal) * idepthScale;
			tmp.x() = tmp.z() * (static_cast<float>(col) - cx) * ifx;
			tmp.y() = tmp.z() * (static_cast<float>(row) - cy) * ify;
			keypoints3D->push_back(tmp);
		}
	}
	assert(keypoints->size() == keypoints3D->size());
}

void Frame::setAverageDescriptors()
{
	assert(descriptors);
	int type = descriptors->type();

  // start a new piece of memory to store the average descriptor
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

  // std::cout << "average descriptor size " << averageFeatureDescriptor->size() << "  " << averageMat->size() << std::endl;
	averageFeatureDescriptor = averageMat;
}

}
