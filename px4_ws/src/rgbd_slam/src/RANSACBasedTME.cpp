/*
 * RANSACBasedTME.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#include <limits>
#include <assert.h>
#include <iostream>
 
#include "AnalyticBasedTME.h"
#include "RANSACBasedTME.h"

#include "Frame.h"

bool RANSACBasedTME::estimateTransformationMatrix(  // return true if transformation matrix is valid and false if invalid
		const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
		const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
		const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
		const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
		const std::vector<int>& consensus, // in: index of the matches in the consensus
		Eigen::Matrix4f& transformMat // out: transformation matrix to transform the input to the target
		)
{
	assert(consensus.size() >= 3);

	// initial test
	for(int i = 1; i < static_cast<int>(consensus.size()); ++i)
	{
		int curCons = consensus.at(i);
		int prevCons = consensus.at(i-1);
		float df = (keys3D1.at(matchIdx1.at(curCons)) - keys3D1.at(matchIdx1.at(prevCons))).norm();//distance to the previous query point
		float dt = (keys3D2.at(matchIdx2.at(curCons)) - keys3D2.at(matchIdx2.at(prevCons))).norm();//distance from one to the next train point
		if ( fabs(df-dt) > maxDistInitialTest )
			return false;
	}

	// least-squares transformation parameter estimation
	AnalyticBasedTME::estimate(keys3D1, matchIdx1, keys3D2, matchIdx2, consensus, transformMat);

	return true;
}

int RANSACBasedTME::computeSquaredDistance(
					const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
					const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
					const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
					const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
					const Eigen::Matrix4f& transformMat,
					const float maxSquaredDist,
					std::vector<int>& consensus, // out: index of the matches in the new consensus
					float& mse
					)
{
	assert(matchIdx1.size() == matchIdx2.size());

	mse = 0;
	consensus.clear();

	for (int point = 0; point < static_cast<int>(matchIdx1.size()); ++point)
	{
		const Eigen::Vector3f& source = keys3D1.at(matchIdx1.at(point));
		const Eigen::Vector3f& target = keys3D2.at(matchIdx2.at(point));
		const float squaredDist = ( (transformMat.topLeftCorner<3,3>()*source + transformMat.topRightCorner<3,1>()) - target).squaredNorm();

		if (maxSquaredDist > squaredDist)
		{
			consensus.push_back(point);
			mse += squaredDist;
		}
	}

	if (!consensus.empty())
		mse /= static_cast<float>(consensus.size());
	else
		mse = std::numeric_limits<float>::infinity();

	return static_cast<int>(consensus.size());
}

bool RANSACBasedTME::estimateTrafo( // return false if estimation failed and else ture.
				const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
				const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
				const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
				const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
				Eigen::Matrix4f& transformMat, // out: transformation matrix to transform the input to the target
				Eigen::Matrix<float, 6, 6>& informationMat // out: information matrix of the transformation
				) // Note: This function must be thread safe!!!
{
	assert(matchIdx1.size() == matchIdx2.size());

	std::vector<int> consensus(Frame::maxNrOfKeyPoints);

	const int nrOfPts = static_cast<int>(matchIdx1.size());
	const float nrOfPtsF = static_cast<float>(nrOfPts);
	const float invNrOfPtsF = 1.0f/static_cast<float>(nrOfPts);
	transformMat = Eigen::Matrix4f::Identity();

	if (nrOfPts < 3)
		return false;

	int optInlier = 0;
	float optMse = std::numeric_limits<float>::infinity();

	{
		//
		// Initial guess --> no movement (transMat == identity)
		//
		float mse;

		int inlier = computeSquaredDistance(keys3D1, matchIdx1, keys3D2, matchIdx2, transformMat, squaredMaxDistInlier, consensus, mse);
		if (inlier > minInlier && mse < squaredMaxDistInlier)
		{
			optMse = mse;
			optInlier = inlier;
		}
	}

	Eigen::Matrix4f transfMat;
	Eigen::Matrix4f refinedTransfMat;
	for (int n = 0; n < iter; ++n)
	{
		int idx[3] = { };
		while (idx[1] == idx[2] || idx[0] == idx[2] || idx[0] == idx[1])
		{
			idx[0] = rand() % nrOfPts;
			idx[1] = rand() % nrOfPts;
			idx[2] = rand() % nrOfPts;
		}

		consensus.clear();
		consensus.push_back(idx[0]);
		consensus.push_back(idx[1]);
		consensus.push_back(idx[2]);

		if (!estimateTransformationMatrix(keys3D1, matchIdx1, keys3D2, matchIdx2, consensus, transfMat))
			continue; // transformation matrix estimation failed

		int inlierRefined = 0;
		float mseRefined = std::numeric_limits<float>::infinity();
		for(int refine = 4; refine < 20; ++refine)
		{
			float mse;
			int inlier = computeSquaredDistance(keys3D1, matchIdx1, keys3D2, matchIdx2, transfMat, squaredMaxDistInlier*(4.0f/static_cast<float>(refine)), consensus, mse);

			if (inlier < minInlier || mse > squaredMaxDistInlier)
				break; // no refinement needed in this case

			if (inlier > inlierRefined && mse < mseRefined)
			{
				if (!estimateTransformationMatrix(keys3D1, matchIdx1, keys3D2, matchIdx2, consensus, transfMat))
					break; // transformation matrix estimation failed

				// save refined parameters
				inlierRefined = inlier;
				mseRefined = mse;
				refinedTransfMat = transfMat;
			}
			else
				break;
		}

		if (inlierRefined > optInlier && mseRefined < optMse)
		{
			optInlier = inlierRefined;
			optMse = mseRefined;
			transformMat = refinedTransfMat;
			if (termInlierPct < static_cast<float>(optInlier)*invNrOfPtsF)
				break;
		}
	}

	informationMat = Eigen::Matrix<float, 6, 6>::Identity() * (static_cast<float>(optInlier) / optMse);

	//std::cout << "information matrix" << std::endl<< informationMat << std::endl;

	return (optInlier >= minInlier && squaredMaxDistInlier > optMse);
}
