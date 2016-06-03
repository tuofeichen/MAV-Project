 /**
 * @file TestAnalyticBasedTME.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file tests the analytic based transformation estimator
 *
 */

#include <iostream>
#include <string>

#include "pcl/common/transforms.h"
#include "pcl/console/time.h"  // TicToc

#include "RANSACBasedTME.h"

using namespace std;
using namespace SLAM;

void print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

enum{testCases = 7};
const int cloudSize[testCases] = {0, 10, 100, 250, 500, 750, 1000};
const bool outputRun[testCases] = {false, true, true, true, true, true, true};

const double allowedAcc = 5e-6;
const double transformMatAcc[testCases] = {0, allowedAcc, allowedAcc, allowedAcc, allowedAcc, allowedAcc, allowedAcc};

const double allowedAccNoise = 2e-3;
const double transformMatNoiseAcc[testCases] = {0, allowedAccNoise, allowedAccNoise, allowedAccNoise, allowedAccNoise, allowedAccNoise, allowedAccNoise};
const double transformMatOutlierAcc[testCases] = {0, allowedAcc, allowedAcc, allowedAcc, allowedAcc, allowedAcc, allowedAcc};
const double transformMatNoiseOutlierAcc[testCases] = {0, allowedAccNoise, allowedAccNoise, allowedAccNoise, allowedAccNoise, allowedAccNoise, allowedAccNoise};

// transformation
const double theta = M_PI / 8;  // The angle of rotation in radians
const double tX = 0.03;
const double tY = 0.05;
const double tZ = 0.10;

// distances
const float near = 0.4; // 0.4 m
const float far = 8.0; // 8 m

// noise
const float sdevX = 0.005; // 0.5 cm
const float meanX = 0.0; // 0

const float sdevY = 0.005; // 0.5 cm
const float meanY = 0.0; // 0

const float sdevZ = 0.005; // 0.5 cm
const float meanZ = 0.0; // 0

// outlier
float outlierProb = 0.40; // 40% outlier

int main()
{
	pcl::console::TicToc time;

	// The point clouds we will be using
	std::vector<Eigen::Vector3f> cloud_in;
	std::vector<Eigen::Vector3f> cloud_tr;
	std::vector<Eigen::Vector3f> cloud_trNoise;
	std::vector<Eigen::Vector3f> cloud_trOutlier;
	std::vector<Eigen::Vector3f> cloud_trNoiseOutlier;

	std::vector<int> matches;

	// Defining a rotation matrix and translation vector
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity ();

	// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	transformation_matrix (0, 0) = cos (theta);
	transformation_matrix (0, 1) = -sin (theta);
	transformation_matrix (1, 0) = sin (theta);
	transformation_matrix (1, 1) = cos (theta);

	// A translation on Z axis (0.4 meters)
	transformation_matrix (0, 3) = tX; //x
	transformation_matrix (1, 3) = tY; //y
	transformation_matrix (2, 3) = tZ; //z

	// Display in terminal the transformation matrix
	std::cout << "Applying the following rigid transformation:" << std::endl;
	print4x4Matrix (transformation_matrix);

	// Start test
	RANSACBasedTME tme(10000, 0.02, 0.03, 0.60, 30);
	Eigen::Matrix<double, 6, 6> infoMat;
	for(int test = 0; test < testCases; ++test)
	{
		// Fill in the CloudIn data
		cloud_in.clear();
		matches.clear();
		std::cout << std::endl << "Point cloud size: " << cloudSize[test] << std::endl;
		for (int i = 0; i < cloudSize[test]; ++i)
		{
			Eigen::Vector3f tmp;
			tmp.x() = static_cast<float>(rand()) / static_cast<float>(RAND_MAX + 1.0) * (far - near) + near;
			tmp.y() = static_cast<float>(rand()) / static_cast<float>(RAND_MAX + 1.0) * (far - near) + near;
			tmp.z() = static_cast<float>(rand()) / static_cast<float>(RAND_MAX + 1.0) * (far - near) + near;
			cloud_in.push_back(tmp);
			matches.push_back(i);
		}

		// Executing the transformation
		cloud_tr.clear();
		cloud_trNoise.clear();
		cloud_trOutlier.clear();
		cloud_trNoiseOutlier.clear();
		for(int i = 0; i < cloudSize[test]; ++i)
		{
			Eigen::Vector3f tr = transformation_matrix.topLeftCorner<3,3>() * cloud_in.at(i) + transformation_matrix.topRightCorner<3,1>();
			cloud_tr.push_back(tr);
			cloud_trNoise.push_back(tr);
			cloud_trOutlier.push_back(tr);
			cloud_trNoiseOutlier.push_back(tr);
		}

		// Add noise and outlier
		int outlierCnt = 0;
		for (int i = 0; i < cloudSize[test]; ++i)
		{
			// create gaussian noise for x,y and z
			float v1, v2, s;
			do {
				v1 = 2 * static_cast<float> (rand()) / RAND_MAX - 1.0;
				v2 = 2 * static_cast<float> (rand()) / RAND_MAX - 1.0;
				s = v1 * v1 + v2 * v2;
			} while (s >= 1.0 || s == 0.0);
			float noiseX = v1 * sqrt(-2.0 * log(s) / s) * sdevX + meanX;

			do {
				v1 = 2 * static_cast<float> (rand()) / RAND_MAX - 1.0;
				v2 = 2 * static_cast<float> (rand()) / RAND_MAX - 1.0;
				s = v1 * v1 + v2 * v2;
			} while (s >= 1.0 || s == 0.0);
			float noiseY = v1 * sqrt(-2.0 * log(s) / s) * sdevY + meanY;

			do {
				v1 = 2 * static_cast<float> (rand()) / RAND_MAX - 1.0;
				v2 = 2 * static_cast<float> (rand()) / RAND_MAX - 1.0;
				s = v1 * v1 + v2 * v2;
			} while (s >= 1.0 || s == 0.0);
			float noiseZ = v1 * sqrt(-2.0 * log(s) / s) * sdevZ + meanZ;

			// add noise
			cloud_trNoise.at(i).x() += noiseX;
			cloud_trNoise.at(i).y() += noiseY;
			cloud_trNoise.at(i).z() += noiseZ;

			cloud_trNoiseOutlier.at(i).x() += noiseX;
			cloud_trNoiseOutlier.at(i).y() += noiseY;
			cloud_trNoiseOutlier.at(i).z() += noiseZ;

			// add outlier
			if ((static_cast<float>(rand()) / static_cast<float>(RAND_MAX + 1.0)) < outlierProb)
			{
				++outlierCnt;

				float outlier = static_cast<float>(rand()) / static_cast<float>(RAND_MAX + 1.0) * (far - near) + near;

				cloud_trOutlier.at(i).x() += outlier;
				cloud_trOutlier.at(i).y() += outlier;
				cloud_trOutlier.at(i).z() += outlier;

				cloud_trNoiseOutlier.at(i).x() += outlier;
				cloud_trNoiseOutlier.at(i).y() += outlier;
				cloud_trNoiseOutlier.at(i).z() += outlier;
			}

		}

		//
		// run transformation matrix estimation for points
		//
		time.tic ();
		Eigen::Matrix4f tm;
		bool ret = tme.estimateTrafo( cloud_in, matches, cloud_tr, matches, tm, infoMat);

		// check
		cout << "Estimation took " << time.toc () << " ms" << " of test case nr. " << test <<  ": ";
		if (ret == outputRun[test])
			cout << "||| output test PASSED ||| ";
		else
			cout << "||| output test FAILED ||| ";

		if (ret)
		{
			if(tm.isApprox(transformation_matrix,transformMatAcc[test]))
				cout << "transformation matrix PASSED ||| " << endl;
			else
			{
				cout << "transformation matrix FAILED ||| " << endl;
				print4x4Matrix (tm);
			}
		}
		else
			cout << endl;

		//
		// run transformation matrix estimation for points with noise
		//
		time.tic ();
		Eigen::Matrix4f tmNoise;
		ret = tme.estimateTrafo( cloud_in, matches, cloud_trNoise, matches, tmNoise, infoMat);

		// check
		cout << "Estimation took " << time.toc () << " ms " << "of test case nr. " << test <<  " (with noise): ";
		if (ret == outputRun[test])
			cout << "||| output test PASSED ||| ";
		else
			cout << "||| output test FAILED ||| ";

		if (ret)
		{
			if(tmNoise.isApprox(transformation_matrix, transformMatNoiseAcc[test]))
				cout << "transformation matrix PASSED ||| " << endl;
			else
			{
				cout << "transformation matrix FAILED ||| " << endl;
				print4x4Matrix (tmNoise);
			}
		}
		else
			cout << endl;

		//
		// run transformation matrix estimation for points with outliers
		//
		time.tic ();
		Eigen::Matrix4f tmOutliers;
		ret = tme.estimateTrafo( cloud_in, matches, cloud_trOutlier, matches, tmOutliers, infoMat);

		// check
		cout << "Estimation took " << time.toc () << " ms " << "of test case nr. " << test <<  " (with " << outlierCnt << " outliers): ";
		if (ret == outputRun[test])
			cout << "||| output test PASSED ||| ";
		else
			cout << "||| output test FAILED ||| ";

		if (ret)
		{
			if(tmOutliers.isApprox(transformation_matrix,transformMatOutlierAcc[test]))
				cout << "transformation matrix PASSED ||| " << endl;
			else
			{
				cout << "transformation matrix FAILED ||| " << endl;
				print4x4Matrix (tmOutliers);
			}
		}
		else
			cout << endl;


		//
		// run transformation matrix estimation for points with noise and outliers
		//
		time.tic ();
		Eigen::Matrix4f tmNoiseOutliers;
		ret = tme.estimateTrafo( cloud_in, matches, cloud_trNoiseOutlier, matches, tmNoiseOutliers, infoMat);

		// check
		cout << "Estimation took " << time.toc () << " ms " << "of test case nr. " << test <<  "(with noise and " << outlierCnt << " outliers): ";
		if (ret == outputRun[test])
			cout << "||| output test PASSED ||| ";
		else
			cout << "||| output test FAILED ||| ";

		if (ret)
		{
			if(tmNoiseOutliers.isApprox(transformation_matrix,transformMatNoiseOutlierAcc[test]))
				cout << "transformation matrix PASSED ||| " << endl;
			else
			{
				cout << "transformation matrix FAILED ||| " << endl;
				print4x4Matrix (tmNoiseOutliers);
			}
		}
		else
			cout << endl;
	}

	return 0;
}
