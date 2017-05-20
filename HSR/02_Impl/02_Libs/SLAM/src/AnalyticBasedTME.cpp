/**
 * @file AnalyticBasedTME.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the AnalyticBasedTME class.
 *
 */
#include "Eigen/SVD"
#include "Eigen/Eigen"

#include "AnalyticBasedTME.h"

//typedef  double FloatingPoint;
//typedef  Eigen::Vector4d Vector4;
//typedef  Eigen::Vector3d Vector3;
//typedef  Eigen::Matrix3d Matrix3;

typedef  float FloatingPoint;
typedef  Eigen::Vector4f Vector4;
typedef  Eigen::Vector3f Vector3;
typedef  Eigen::Matrix3f Matrix3;

namespace SLAM {

void AnalyticBasedTME::estimate( // return false if estimation failed and else ture.
			const std::vector<Eigen::Vector3f>& keys3D1, // in: 3D keypoints of image 1
			const std::vector<int>& matchIdx1, // in: matched keypoint index of image 1
			const std::vector<Eigen::Vector3f>& keys3D2, // in: 3D keypoints of image 2
			const std::vector<int>& matchIdx2, // in: matched keypoint index of image 2
			const std::vector<int>& activeIdx, // in: active index of the matches
			Eigen::Matrix4f& transformMat // out: transformation matrix to transform the input to the target
			) // Note: This function must be thread safe!!!
{
	assert(matchIdx1.size() == matchIdx2.size());
	assert(matchIdx1.size() >= 3);

	const FloatingPoint scaleFactor = 1.0 / static_cast<FloatingPoint>(activeIdx.size());

	transformMat = Eigen::Matrix4f::Identity();

	// compute mean of x and y
	Vector3 meanX = Vector3::Zero();
	Vector3 meanY = Vector3::Zero();
	for (int i = 0; i < static_cast<int>(activeIdx.size()); ++i)
	{
		meanX += keys3D1.at(matchIdx1.at(activeIdx.at(i)));
//		meanX += keys3D1.at(matchIdx1.at(activeIdx.at(i))).cast<double>();

		meanY += keys3D2.at(matchIdx2.at(activeIdx.at(i)));
//		meanY += keys3D2.at(matchIdx2.at(activeIdx.at(i))).cast<double>();
	}
	meanX *= scaleFactor;
	meanY *= scaleFactor;

	// calculate covariance and variance of x
	Matrix3 covXY = Matrix3::Zero();
	FloatingPoint varX = 0;
	for (int i = 0; i < static_cast<int>(activeIdx.size()); ++i)
	{
		Vector3 x = keys3D1.at(matchIdx1.at(activeIdx.at(i))) - meanX;
//		Vector3 x = keys3D1.at(matchIdx1.at(activeIdx.at(i))).cast<double>() - meanX;
		varX += x.squaredNorm();

		Vector3 y = keys3D2.at(matchIdx2.at(activeIdx.at(i))) - meanY;
//		Vector3 y = keys3D2.at(matchIdx2.at(activeIdx.at(i))).cast<double>() - meanY;
		covXY += y*x.transpose();
	}
	varX *= scaleFactor;
	covXY *= scaleFactor;

	// single value decomposition of covariance_xy
	Eigen::JacobiSVD<Matrix3> svd(covXY, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Matrix3 s = Matrix3::Identity();
	FloatingPoint scale = 1.0/varX;
	const FloatingPoint detCovXY = covXY.determinant();
	if(fabs(detCovXY) < 1e-12)
	{
		const FloatingPoint detUV = svd.matrixU().determinant() * svd.matrixV().determinant();
		if (detUV < 0)
		{
			s(2,2) = -1;
			scale *= svd.singularValues().middleRows(0,2).sum() - svd.singularValues()(2);
		}
		else
			scale *= svd.singularValues().sum();
	}
	else if (detCovXY < 0)
	{
		s(2,2) = -1;
		scale *= svd.singularValues().middleRows(0,2).sum() - svd.singularValues()(2);
	}
	else
		scale *= svd.singularValues().sum();

	// calculate the rotation, translation and scaling
	Matrix3 rot = svd.matrixU() * s * svd.matrixV().transpose(); // rotation
	Vector3 trans = meanY - scale*rot*meanX; // translation

	// comment out when using double
	transformMat.topLeftCorner<3,3>() = rot;
	transformMat.topRightCorner<3,1>() = trans;

	// comment in when using double
//	transformMat.topLeftCorner<3,3>() = rot.cast<float>();
//	transformMat.topRightCorner<3,1>() = trans.cast<float>();
}

} /* namespace HAL */
