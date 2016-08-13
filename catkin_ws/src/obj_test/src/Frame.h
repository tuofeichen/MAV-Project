 /**
 * @file Frame.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the Frame class
 *
 */
#ifndef INCLUDES_FRAME_H_
#define INCLUDES_FRAME_H_

#include <assert.h>
#include <vector>

#include <boost/smart_ptr/shared_ptr.hpp>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"

#include "Eigen/Core"

namespace SLAM {

/**
 * @class Frame Frame.h "Frame.h"
 * @brief The Frame stores all important informations about a rgb-d frame
 */
class Frame
{
public:

	//
	// Asus
	static constexpr float depthScale = 1000.0; ///< Depth scale factor for the asus xtion pro live sensor

	// VGA
//	static constexpr float fx = 570.0; ///< focal length in x direction in pixel
//	static constexpr float fy = 570.0; ///< focal length in y direction in pixel
//	static constexpr float cx = 319.5; ///< center point in x direction in pixel
//	static constexpr float cy = 239.5; ///< center point in y direction in pixel

	// QVGA
	static constexpr float fx = 285.0; ///< focal length in x direction in pixel
	static constexpr float fy = 285.0; ///< focal length in y direction in pixel
	static constexpr float cx = 159.5; ///< center point in x direction in pixel
	static constexpr float cy = 119.5; ///< center point in y direction in pixel

	enum{
		rows = 240,
		cols = 320,
		depthMode = 0,  // select index=0 320x240, 30 fps, 1mm
		colorMode = 0 // select index 0: 320x240, 30 fps, 200 format (RGB)
	};
	
	boost::shared_ptr<double> setTime() { return time; }
	boost::shared_ptr<cv::Mat> setRgb() { return rgb; }
	boost::shared_ptr<cv::Mat> setGray() { return gray; }
	boost::shared_ptr<cv::Mat> setDepth() { return depth; }


	//
	// Kinect V1 datasets
//	static constexpr float fx = 525.0; ///< focal length in x direction in pixel
//	static constexpr float fy = 525.0; ///< focal length in y direction in pixel
//	static constexpr float cx = 319.5; ///< center point in x direction in pixel
//	static constexpr float cy = 239.5; ///< center point in y direction in pixel
//	static constexpr float depthScale = 5000.0; ///< Depth scale factor for the kinect v1 datasets

	//
	// Kinect V2
//	static constexpr float fx = 365.2371; ///< focal length in x direction in pixel
//	static constexpr float fy = 365.2371; ///< focal length in y direction in pixel
//	static constexpr float cx = 259.393; ///< center point in x direction in pixel
//	static constexpr float cy = 205.7591; ///< center point in y direction in pixel
//	static constexpr float depthScale = 1000.0; ///< Depth scale factor for the kinect v2 datasets

	/**
	 * @brief Frame Constructor
	 */
	Frame();

	/**
	 * @brief Constructor
	 *
	 * @param rgbImage the rgb image of the frame (in)
	 * @param grayImage the gray image of the frame (in)
	 * @param depthImage the depth image of the frame (in)
	 * @param timeStamp the time stamp of the frame (in)
	 *
	 */
	Frame(boost::shared_ptr<cv::Mat>& rgbImage, boost::shared_ptr<cv::Mat>& grayImage, boost::shared_ptr<cv::Mat>& depthImage, boost::shared_ptr<double>& timeStamp);

	/**
	 * @breif Destructor
	 */
	~Frame() { }

	/**
	 * @breif getId returns the id of the frame in the pose graph
	 *
	 * @return returns the id
	 */
	int getId() const { assert(id); return *id; }

	/**
	 * @breif sets the id of the frame in the pose graph
	 *
	 * @param i the id
	 */
	void setId(int i) { assert(id); *id = i; }

	/**
	 * @breif returns if the frame is a key frame or not
	 *
	 * @return returns true if the frame is a key frame
	 */
	bool getKeyFrameFlag() const { assert(keyFrameFlag); return *keyFrameFlag; }

	/**
	 * @brief sets the key frame flag
	 *
	 * @param flag key frame flag (in)
	 */
	void setKeyFrameFlag(bool flag) { assert(keyFrameFlag); *keyFrameFlag = flag; }

	/**
	 * @breif returns if the frame is a dummy frame or not
	 *
	 * @return returns true if the frame is a dummy frame
	 */
	bool getDummyFrameFlag() const { assert(dummyFrameFlag); return *dummyFrameFlag; }

	/**
	 * @brief sets the dummy frame flag
	 *
	 * @param flag dummy frame flag (in)
	 */
	void setDummyFrameFlag(bool flag) { assert(dummyFrameFlag); *dummyFrameFlag = flag; }

	/**
	 * @breif getTime gets the time stamp
	 *
	 * @return returns time stamp
	 */
	const double& getTime() const { assert(time); return *time; }

	/**
	 * @breif getRgb gets RGB image
	 *
	 * @return returns RGB image
	 */
	const cv::Mat& getRgb() const { assert(rgb); return *rgb; }

	/**
	 * @breif getGray gets gray image
	 *
	 * @return returns gray image
	 */
	const cv::Mat& getGray() const { assert(gray); return *gray; }

	/**
	 * @breif getDepth gets depth image
	 *
	 * @return returns depth image
	 */
	const cv::Mat& getDepth() const { assert(depth); return *depth; }

	/**
	 * @breif deletes the rgb image
	 */
	void deleteRgb() { rgb.reset(); }

	/**
	 * @breif deletes the gray image
	 */
	void deleteGray() { gray.reset(); }

	/**
	 * @breif deletes the depth image
	 */
	void deleteDepth() { depth.reset(); }

	/**
	 * @breif setKeypoints sets keypoints of the image
	 *
	 * @param keys found feature points in the image (input)
	 */
	void setKeypoints(boost::shared_ptr<std::vector<cv::KeyPoint>> keys);

	/**
	 * @breif getKeypoints gets keypoints of the image
	 *
	 * @return returns key points
	 */
	const std::vector<cv::KeyPoint>& getKeypoints() const { assert(keypoints); return *keypoints; }

	/**
	 * @breif getKeypoints3D gets 3D keypoints of the image
	 *
	 * @return returns 3D key points
	 */
	const std::vector<Eigen::Vector3f>& getKeypoints3D() const { assert(keypoints3D); return *keypoints3D; }

	/**
	 * @breif setDescriptors sets feature descriptors of the image
	 *
	 * @param keys found feature descriptors of the image (input)
	 */
	void setDescriptors(boost::shared_ptr<cv::Mat> descs) { descriptors = descs; setAverageDescriptors(); }

	/**
	 * @breif getDescriptors gets feature descriptors of the image
	 *
	 * @return returns feature descriptor
	 */
	const cv::Mat& getDescriptors() const { assert(descriptors); return *descriptors; }

	/**
	 * @breif getAverageDescriptors gets the average feature descriptor of the image
	 *
	 * @return returns average feature descriptor
	 */
	const cv::Mat& getAverageDescriptors() const { assert(averageFeatureDescriptor); return *averageFeatureDescriptor; }

private:
	void setAverageDescriptors();

	boost::shared_ptr<int> id;
	boost::shared_ptr<bool> keyFrameFlag;
	boost::shared_ptr<bool> dummyFrameFlag;
	boost::shared_ptr<double> time;
	boost::shared_ptr<cv::Mat> rgb;
	boost::shared_ptr<cv::Mat> gray;
	boost::shared_ptr<cv::Mat> depth;
	boost::shared_ptr<std::vector<cv::KeyPoint>> keypoints;
	boost::shared_ptr<std::vector<Eigen::Vector3f>> keypoints3D;
	boost::shared_ptr<cv::Mat> descriptors;
	boost::shared_ptr<cv::Mat> averageFeatureDescriptor;

	static constexpr float ifx = 1.0/fx;
	static constexpr float ify = 1.0/fy;
	static constexpr float idepthScale = 1.0/depthScale;
};

}
#endif /* INCLUDES_FRAME_H_ */
