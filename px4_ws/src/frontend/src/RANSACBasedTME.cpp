/**
 * @file RANSACBasedTME.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the RANSACBasedTME class.
 *
 */

#include <limits>
#include <assert.h>
#include <iostream>
#include "Eigen/Geometry"

#include "AnalyticBasedTME.h"
#include "RANSACBasedTME.h"

#include <ctime>

namespace SLAM {

#ifdef WITH_MAHALANOBIS
const double RANSACBasedTME::rasterStddevX = 3*tan(camAngleX/camResolX);  //5pix stddev in x
const double RANSACBasedTME::rasterStddevY = 3*tan(camAngleY/camResolY);  //5pix stddev in y
const double RANSACBasedTME::rasterCovX = rasterStddevX * rasterStddevX;
const double RANSACBasedTME::rasterCovY = rasterStddevY * rasterStddevY;
#endif

RANSACBasedTME::RANSACBasedTME(int iterations, float maxDistForInlier, float maxDistanceInitialTest, float terminationInlierPct, int inlierMin)
: iter(iterations),
  thresholdAbsolutDistanceTest(maxDistanceInitialTest),
  squaredMaxDistInlier(maxDistForInlier*maxDistForInlier),
  minInlier(inlierMin), termInlierPct(terminationInlierPct)
{
	srand (std::time(NULL));
}

RANSACBasedTME::~RANSACBasedTME()
{ }

bool RANSACBasedTME::estimateTransformationMatrix(  // return true if transformation matrix is valid and false if invalid
		const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
		const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
		const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
		const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
		const std::vector<int>& consensus, // in: index of the matches in the consensus
		Eigen::Matrix4f& transformMat // out: transformation matrix to transform the input to the target
		) const
{
	assert(consensus.size() >= 3);
	assert(matchIdx1.size() == matchIdx2.size());

	// initial test
	for(int i = 1; i < static_cast<int>(consensus.size()); ++i)
	{
		int curCons = consensus.at(i);
		int prevCons = consensus.at(i-1);

		float df = (keys3D1.at(matchIdx1.at(curCons)) - keys3D1.at(matchIdx1.at(prevCons))).norm();//distance to the previous query point
		float dt = (keys3D2.at(matchIdx2.at(curCons)) - keys3D2.at(matchIdx2.at(prevCons))).norm();//distance from one to the next train point
		
		if ( fabs(df-dt) > thresholdAbsolutDistanceTest )
			return false;
	}

	// least-squares transformation parameter estimation
	AnalyticBasedTME::estimate(keys3D1, matchIdx1, keys3D2, matchIdx2, consensus, transformMat);

	return true;
}

#ifdef WITH_MAHALANOBIS

double RANSACBasedTME::mahalanobisDistance(const Eigen::Vector4f& x1,
                      const Eigen::Vector4f& x2,
                      const Eigen::Matrix4f& tf_1_to_2) const
{
  Eigen::Vector4d x_1 = x1.cast<double>();
  x_1(3) = 1.0;
  Eigen::Vector4d x_2 = x2.cast<double>();
  x_2(3) = 1.0;
  Eigen::Matrix4d tf_12 = tf_1_to_2.cast<double>();

  Eigen::Vector3d mu_1 = x_1.head<3>();
  Eigen::Vector3d mu_2 = x_2.head<3>();

  Eigen::Vector3d mu_1_in_frame_2 = (tf_12 * x_1).head<3>(); // μ₁⁽²⁾  = T₁₂ μ₁⁽¹⁾
  Eigen::Matrix3d rotation_mat = tf_12.block(0,0,3,3);

  //Point 1
  Eigen::Matrix3d cov1 = Eigen::Matrix3d::Zero();
  cov1(0,0) = rasterCovX * mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(1,1) = rasterCovY * mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(2,2) = depthCovariance(mu_1(2));

  //Point2
  Eigen::Matrix3d cov2 = Eigen::Matrix3d::Zero();
  cov2(0,0) = rasterCovX* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(1,1) = rasterCovY* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(2,2) = depthCovariance(mu_2(2));

  Eigen::Matrix3d cov1_in_frame_2 = rotation_mat.transpose() * cov1 * rotation_mat;//Works since the cov is diagonal => Eig-Vec-Matrix is Identity

  // Δμ⁽²⁾ =  μ₁⁽²⁾ - μ₂⁽²⁾
  Eigen::Vector3d delta_mu_in_frame_2 = mu_1_in_frame_2 - mu_2;
  delta_mu_in_frame_2(2) = delta_mu_in_frame_2(2);
  // Σc = (Σ₁ + Σ₂)
  Eigen::Matrix3d cov_mat_sum_in_frame_2 = cov1_in_frame_2 + cov2;
  //ΔμT Σc⁻¹Δμ
  double sqrd_mahalanobis_distance = delta_mu_in_frame_2.transpose() * cov_mat_sum_in_frame_2.inverse() * delta_mu_in_frame_2;

  if(!(sqrd_mahalanobis_distance >= 0.0))
  {
    return std::numeric_limits<double>::max();
  }
  return sqrd_mahalanobis_distance;
}

#endif

int RANSACBasedTME::computeSquaredDistance(
					const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
					const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
					const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
					const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
					const Eigen::Matrix4f& transformMat,
					const float maxSquaredDist,
					std::vector<int>& consensus, // out: index of the matches in the new consensus
					float& mse
					) const
{
	assert(matchIdx1.size() == matchIdx2.size());

	mse = 0;
	consensus.clear();

	for (int point = 0; point < static_cast<int>(matchIdx1.size()); ++point)
	{

#ifdef WITH_MAHALANOBIS
		Eigen::Vector4f source = Eigen::Vector4f::Ones();
		source.head<3>() = keys3D1.at(matchIdx1.at(point));
		Eigen::Vector4f target = Eigen::Vector4f::Ones();
		target.head<3>() = keys3D2.at(matchIdx2.at(point));
		const float squaredDist = static_cast<float>(mahalanobisDistance(source, target, transformMat));
#else
		const Eigen::Vector3f& source = keys3D1.at(matchIdx1.at(point));
		const Eigen::Vector3f& target = keys3D2.at(matchIdx2.at(point));
		const float squaredDist = ( (transformMat.topLeftCorner<3,3>()*source + transformMat.topRightCorner<3,1>()) - target).squaredNorm();
#endif

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
				Eigen::Matrix<double, 6, 6>& informationMat, // out: information matrix of the transformation
				std::vector<int>& finalConsensus
				) const // Note: This function must be thread safe!!!
{
	assert(matchIdx1.size() == matchIdx2.size());

	const int nrOfPts = static_cast<int>(matchIdx1.size());
//	const double nrOfPtsD = static_cast<double>(nrOfPts);
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
		std::vector<int> consensus;

		int inlier = computeSquaredDistance(keys3D1, matchIdx1, keys3D2, matchIdx2, transformMat, squaredMaxDistInlier, consensus, mse);
		if (inlier > minInlier && mse < squaredMaxDistInlier)
		{
			optMse = mse;
			optInlier = inlier; //shouldn' here return ? 
		}
	}

	std::vector<int> consensus;
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

//			idx[0] = static_cast<int>(static_cast<double>(rand()) / (static_cast<double>(RAND_MAX)+1.0) * nrOfPtsD);
//			idx[1] = static_cast<int>(static_cast<double>(rand()) / (static_cast<double>(RAND_MAX)+1.0) * nrOfPtsD);
//			idx[2] = static_cast<int>(static_cast<double>(rand()) / (static_cast<double>(RAND_MAX)+1.0) * nrOfPtsD);
		}

		consensus.clear();
		// every iteration start by three points and refine it 
		consensus.push_back(idx[0]);
		consensus.push_back(idx[1]);
		consensus.push_back(idx[2]);

		if (!estimateTransformationMatrix(keys3D1, matchIdx1, keys3D2, matchIdx2, consensus, transfMat))
			continue; //if the three point doesn't work out 

		int inlierRefined = 0;
		bool refinedSucceed = false;
		float mseRefined = std::numeric_limits<float>::infinity();

		// refining the RANSAC ? 
		for(int refine = 4; refine < 20; ++refine)
		{
			float mse;
			int inlier = computeSquaredDistance(keys3D1, matchIdx1, keys3D2, matchIdx2, transfMat, squaredMaxDistInlier*(4.0f/static_cast<float>(refine)), consensus, mse);
			Eigen::Matrix4f tmOld = transfMat;

			if (inlier < minInlier || mse > squaredMaxDistInlier)
				break; // no refinement needed in this case (not worth it)

			if (inlier > inlierRefined && mse < mseRefined)
			{
				refinedSucceed = estimateTransformationMatrix(keys3D1, matchIdx1, keys3D2, matchIdx2, consensus, transfMat);			
				if (!refinedSucceed){ 
					// std::cout << "transfo failed in refinement stage  " << consensus.size()<<std::endl; //shouldn't be terminated? 
					break; // transformation matrix estimation failed
				}
				
				// save refined parameters
				inlierRefined = inlier;
				mseRefined = mse;
				refinedTransfMat = transfMat;
			}
			else
				break;

		}



		if (inlierRefined > optInlier && mseRefined < optMse) // iterative update 
		{
			// std::cout  << " iteration consensus size "<< consensus.size() << std::endl;
			optInlier = inlierRefined;
			optMse = mseRefined;
			transformMat = refinedTransfMat; // last one 

			if (termInlierPct < static_cast<float>(optInlier)*invNrOfPtsF){
				// std::cout <<"# of iteration: " << n << std::endl;
				break;
			}

		}
	}

	informationMat = Eigen::Matrix<double, 6, 6>::Identity() * static_cast<double>(static_cast<float>(optInlier) / optMse);

	// end of the day not necessarily preserved inlier. 
	if (optInlier >= minInlier && squaredMaxDistInlier > optMse)
	{
		// std::cout << "Inlier = " << optInlier << " RMSE = " << sqrt(optMse) << std::endl;
		// finalConsensus = consensus;
		// std::cout << "final consensus size " << consensus.size() << std::endl; 
		return true;
	}
	else
	{

//		std::cout << "Failed to estimate the transformation matrix: Inlier = " << optInlier << " RMSE = " << sqrt(optMse) << std::endl;
		return false;
	}
}

//void RANSACBasedTME::printTransformMatrix(const Eigen::Matrix4f& matrix) const
//{
//  printf ("Rotation :\n");
//  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
//  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
//  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
//  printf ("Translation x, y, z:\n");
//  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
//}

} /* namespace SLAM */
