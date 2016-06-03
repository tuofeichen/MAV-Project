/**
 * @file VisualOdometry.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the VisualOdometry class
 *
 */

#ifndef INCLUDES_VISUALODOMETRY_H_
#define INCLUDES_VISUALODOMETRY_H_

#include "Frame.h"

/**
 * @class VisualOdometry VisualOdometry.h "VisualOdometry.h"
 * @brief The VisualOdometry class  schedules the visual odometry
 */
class VisualOdometry
{
public:
	static constexpr int minNrOfMatches = Frame::minNrOfMatches;
	static constexpr int minNrOfKeyPoints = Frame::minNrOfKeyPoints;

	static constexpr float minTranslation = Frame::minTranslation;
	static constexpr float minRotation = Frame::minRotation;
	static constexpr float maxTranslationPerSecond = Frame::maxTranslationPerSecond;
	static constexpr float maxAngularVelocit = Frame::maxAngularVelocit;

	/**
	 * @breif setKeypoints sets keypoints of the frame
	 *
	 * @param frame frame to apply the visual odometry (input)
	 *
	 * @return returns true if there are enough valid feature points
	 */
	static bool setKeypoints(Frame& frame);
	
	/**
	 * @breif estimateTrafo sets keypoints of the frame
	 *
	 * @param srcFrame source frame (input)
	 * @param targetFrame target frame (input)
	 * @param tmSrcToTarget transformation matrix from source frame to target frame(output)
	 * @param imEstSrcToTarget information matrix from source frame to target frame(output)
	 * @param valid true if the transformation is valid, false otherwise (output)
	 */
	static void estimateTrafo(const Frame& srcFrame, const Frame& targetFrame,
				Eigen::Matrix4f& tmSrcToTarget, Eigen::Matrix<float, 6, 6>& imEstSrcToTarget, bool& valid);
	enum Result {valid, invalid, small};
	
	/**
	 * @breif checkReliability checks if the estimated transformation is reliable
	 *
	 * @param tm transformation matrix (input)
	 * @param dt time difference between timestamps (input)
	 *
	 * @return returns valid if the transformation is reliable, small if the relative movement is to small, invalid otherwise
	 */
	static Result checkReliability(const Eigen::Matrix4f& tm, const double& dt);

private:
	VisualOdometry() { } // no objects

	static constexpr float minSquaredTranslation = minTranslation*minTranslation; // m²
	static constexpr float maxSquaredTranslationPerSecond = maxTranslationPerSecond*maxTranslationPerSecond; // m²/s
};

#endif /* INCLUDES_VISUALODOMETRY_H_ */
