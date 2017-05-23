#include "ObjDetection.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"


#define RANSAC 5
#define MIN_MATCHES 20 // minimum matches for features
#define MAX_DISP_NORM 40


using namespace std;
using namespace cv;


int main()
{

}

ObjDetection::ObjDetection(IFeatures* aFDEM,RosHandler* aRos):
 dem(aFDEM), px4(aRos)
{
  objDetected = 0;
	readTemplate(); // read object template to be matched
}

void ObjDetection::processFrame(Frame newFrame)
{
	objFrame = newFrame; // copy everything

	if (objFrame.getBadFrameFlag() < 1) // should process frame regardless
	   objDetected = objectDetect();

	if (objDetected)
		checkObjAngle(objFrame.getDepth());
	else
  {
		checkForWall(objFrame.getDepth());
  }
	if(px4->getTakeoffFlag() && (!objDetected)) // haven't taken off yet
		checkObstacles(objFrame.getDepth(),20,20,2500); // combine

}

bool  ObjDetection::objectDetect()
{
	std::vector< DMatch > forward_matches;
	std::vector< DMatch > backward_matches;
	std::vector< DMatch > valid_matches;
  	std::vector<Point2f> obj_kpts_matched;
  	std::vector<Point2f> scene_kpts_matched;
  	cv::Mat H; // homography matrix for object detection
	bool objFound = 0;


// symmetry test (we want forward and backward matching to both be above certain threshold)
	if(!dem->match(*tempKeyPoints,*tempDescriptors, objFrame.getKeypoints(), objFrame.getDescriptors(), forward_matches))
	{
		  // cout << "forward match size " << forward_matches.size() << endl;
		  return false;
	}

	else if (!dem->match(objFrame.getKeypoints(), objFrame.getDescriptors(), *tempKeyPoints,*tempDescriptors,backward_matches))
	{

			// cout << "backward match size " << backward_matches.size() << endl << endl;
			return false;
	}
// symmetry test
	for( int i = 0; i < backward_matches.size(); i++ ) // symmetry test
	{
		for (int j = 0; j < forward_matches.size();j++)
		{
			if (backward_matches.at(i).queryIdx == forward_matches.at(j).trainIdx)
				if(backward_matches.at(i).trainIdx == forward_matches.at(j).queryIdx)
				{
					valid_matches.push_back(forward_matches.at(j));
					obj_kpts_matched.push_back( (*tempKeyPoints).at(forward_matches[j].queryIdx).pt );
					scene_kpts_matched.push_back(objFrame.getKeypoints()[forward_matches[j].trainIdx].pt);
					break;
				}
		}
	}

	if (valid_matches.size() < MIN_MATCHES)
		return false;


// Visualization of matching
/*	 Mat img_matches;
	 drawMatches(tempImage, *tempKeyPoints, objFrame.getGray(), objFrame.getKeypoints(), valid_matches, img_matches, Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::DEFAULT );
	 imshow("Object Matching", img_matches);
*/

// find homography
  // H = findHomography( obj_kpts_matched, scene_kpts_matched, CV_RANSAC, RANSAC);
	H = findHomography( obj_kpts_matched, scene_kpts_matched, CV_LMEDS);

	if(!H.empty())
	{
		Mat img_object = tempImage;
		Mat img_scene =  objFrame.getGray();
		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = cvPoint(0,0);
		obj_corners[1] = cvPoint( img_object.cols, 0 );
		obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
		obj_corners[3] = cvPoint( 0, img_object.rows );
		std::vector<Point2f> scene_corners(4);

// This yield a floating point centroid position
		perspectiveTransform(obj_corners, scene_corners, H);
    objCentroid = (scene_corners[0] + scene_corners[1] + scene_corners[2] + scene_corners[3])/4;

		if(objCentroid.x > 4 && objCentroid.x < (objFrame.getGray().cols-4) && objCentroid.y > 4 && objCentroid.y < (objFrame.getGray().rows-4) && norm(objCentroid - prevObjCentroid) < MAX_DISP_NORM)
		{
// for visualization of the homography:
// centroid = ((scene_corners[0] + Point2f( img_object.cols, 0)) + (scene_corners[1] + Point2f( img_object.cols, 0)) + (scene_corners[2] + Point2f( img_object.cols, 0)) + (scene_corners[3] + Point2f( img_object.cols, 0)) )/4;
				// cout << "centroid is " << objCentroid << endl;
				// line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				// line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				// line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				// line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				// circle( img_matches, centroid, 32.0, Scalar( 0, 0, 255 ), 3, 8 );
				// circle( objFrame.getGray(), Point2f((int)objCentroid.x,(int)objCentroid.y), 32.0, Scalar( 255, 0, 0 ), 3, 8 );
				// circle( objFrame.getGray(), objCentroid, 32.0, Scalar( 255, 0, 0 ), 3, 8 );

				// Get depth value if centroid position is in a valid position
				objPoint.z =-1;
				const cv::Mat& Depth = objFrame.getDepth();
				float depthSum = 0;
				int   depthValidCounter = 0;

// take average depth value for the 4x4 neighborhood of centroid
				for(int   l = objCentroid.y - 4; l <objCentroid.y + 4; l++)
				{
					for(int k = objCentroid.x - 4; k <objCentroid.x + 4;k++)
					{
						if(Depth.at<uint16_t>(l,k) != 0 && l > 0 && l < objFrame.getGray().cols && k > 0 && k < objFrame.getGray().rows)
							{
							        depthSum+=Depth.at<uint16_t>(l,k);
						          	depthValidCounter++;
							}
						}
				}

				depthSum = depthSum/((float)depthValidCounter);

				if(depthSum < 3500 && depthSum!=0)
				{
					cout << "x: " << objCentroid.x << endl;
					cout << "y: " << objCentroid.y << endl;
					cout << "depth:  " << depthSum << endl;

					// imshow( "ObjMatches", img_matches );
					objPoint.z = depthSum;
					objPoint.x = objCentroid.x;
					objPoint.y = objCentroid.y;

					px4->updateObjPos(objPoint);
					objFound = 1;
				}
				else
				{ cout << "invalid depth value (found object)"<<endl;}
		}

		prevObjCentroid = objCentroid;
		// past_centroid = centroid;
	}
	return objFound;

}

void ObjDetection::checkObstacles(cv::Mat depth, int d_row, int d_col, int safe_dist) // check specified central region
{
	int minDist = 3500;
	long int totalDist = 0;
	int obstCnt = 0;
	for (int i = -d_row; i< d_row; i++)
	{
		for (int j = -d_col; j< d_col; j++)
		{
			int dist = depth.at<uint16_t>(120+j,160+i);
			if ((dist < safe_dist)&& (dist>0))
			{
				obstCnt++;
				totalDist += dist;
				minDist = std::min(minDist,dist);
      }
		}
	}

// abuse of message here, x y z component doesn't mean anything
	obstacleDistance.x = (float) obstCnt;             // how many violation to minimum distance recorded
	obstacleDistance.y = (float) minDist;             // minimum distance to the obstacle
	obstacleDistance.z = (float) totalDist / obstCnt; // average distance to the obstacle

	px4->updateObstacleDistance (obstacleDistance);
}

void ObjDetection::checkObjAngle(cv::Mat Depth) // these functions needs clean up tbh
{
	int e = 20;
	int square_size = 5;
	int pixel_number_left = 160-e;
	int pixel_number_right = 160+e;
	float r=0;
	float l=0;
	int r_counter=0;
	int l_counter=0;
	int height = 120;
	// float alpha, beta;
	// float angle_per_pixel_rad = (58./320.)*M_PI/180.;
	bool valid_position = true;

	// float FOV = (pixel_number_right - pixel_number_left)*angle_per_pixel_rad;

  for(int k = e; k < 80; k+=10)
	{
    r = 0; l = 0; // reset r and l
    r_counter = 0; l_counter = 0;

    pixel_number_left  = 160 - k;
  	pixel_number_right = 160 + k; // traverse through some k

    for (int i = pixel_number_right; i <  pixel_number_right + square_size; i++)
		{
    	 	for (int j=120-square_size; j< 120+square_size;j++)
				{
					if(Depth.at<uint16_t>(j,i)!=0)
					{
						r += Depth.at<uint16_t>(j,i);
        	  r_counter ++;
					}
				}
		}

 		r = r/(float)r_counter;

		for (int i=pixel_number_left;i<pixel_number_left + square_size;i++)
		{
     		for (int j=120; j< 120+square_size;j++)
				{
					if(Depth.at<uint16_t>(j,i)!=0)
					{
						l += Depth.at<uint16_t>(j,i);
            l_counter ++;
					}
				}
		}
 		l=l/(float)l_counter;

// if average is close enough to the center value then use the average (relatively flat surface)
		if(fabs(r-Depth.at<uint16_t>(120,pixel_number_right)) < 15 && fabs(l-Depth.at<uint16_t>(120,pixel_number_left)) < 15)
		{
      e = k; // note down what's the final distance in between
			valid_position = true;
			break;
		}

	}


  if(valid_position)
	{
    if(fabs(r-l)/max(r,l) < 0.3)
		{
      float D = 2 * e / objFrame.fx; // scale up to real world
      objAngle.x = atan2((l-r) /objFrame.depthScale,D); // in radians
      objAngle.y = objAngle.x*180/M_PI; // in degrees
      objAngle.z =  (r+l)/2; //in mm

      if (objAngle.x>3000||objAngle.y>3000||objAngle.z>3000||objAngle.z ==0)
      {
        	objAngle.x = -5000;
        	objAngle.y = -5000;
          objAngle.z = 0;      // infinite angle value
      }
      else
      {
        cout << "detect object angle " << objAngle.y << endl;
      }
    }
    else
    {
        objAngle.z = -1; // if angle too big
    }

	}
	else
	{
		objAngle.x = -3000;
		objAngle.y = -3000;
		// cout << "Angle too big " << endl;
	}


	px4->updateWallPos (objAngle);
}

void ObjDetection::checkForWall(cv::Mat Depth)
{
   // assume 160,120 is the center of the image
	int e = 25;
	int square_size = 3;
	int pixel_number_left  = 160 - e;
	int pixel_number_right = 160 + e;
	float r = 0;
	float l = 0;

	int r_counter=0;
	int l_counter=0;
	// float alpha, beta;
	// float angle_per_pixel_rad = (58./320.)*M_PI/180.; // 58 is the field of view of the camera (should set as a constant)
	bool valid_position = true;

	// float distance = 0;
	// int   distance_counter = 0;
  //
	// float FOV = (pixel_number_right - pixel_number_left) * angle_per_pixel_rad;

	for(int k = e; k < 80; k+=10)
	{
    r = 0; l = 0; // reset r and l
    r_counter = 0; l_counter = 0;

    pixel_number_left  = 160 - k;
  	pixel_number_right = 160 + k; // traverse through some k

    for (int i = pixel_number_right; i <  pixel_number_right + square_size; i++)
		{
    	 	for (int j=120-square_size; j< 120+square_size;j++)
				{
					if(Depth.at<uint16_t>(j,i)!=0)
					{
						r += Depth.at<uint16_t>(j,i);
        	  r_counter ++;
					}
				}
		}

 		r = r/(float)r_counter;

		for (int i=pixel_number_left;i<pixel_number_left + square_size;i++)
		{
     		for (int j=120; j< 120+square_size;j++)
				{
					if(Depth.at<uint16_t>(j,i)!=0)
					{
						l += Depth.at<uint16_t>(j,i);
            l_counter ++;
					}
				}
		}
 		l=l/(float)l_counter;

// if average is close enough to the center value then use the average (relatively flat surface)
		if(fabs(r-Depth.at<uint16_t>(120,pixel_number_right)) < 15 && fabs(l-Depth.at<uint16_t>(120,pixel_number_left)) < 15)
		{
      e = k; // note down what's the final distance in between
			valid_position = true;
			break;
		}

	}

	if(valid_position)
	{
    if(fabs(r-l)/max(r,l) < 0.3)
		{
      float D = 2 * e / objFrame.fx; // scale up to real world
      wallAngle.x = atan2((l-r) /objFrame.depthScale,D); // in radians
      wallAngle.y = wallAngle.x*180/M_PI; // in degrees
      wallAngle.z = (r + l ) /2; //in mm

      if (wallAngle.x>3000||wallAngle.y>3000||wallAngle.z>3000||wallAngle.z ==0)
      {
        	wallAngle.x = -5000;
        	wallAngle.y = -5000;
          wallAngle.z = 0;      // infinite angle value
      }
    }
    else
    {
        wallAngle.z = -1; // if angle too big
    }

	}
	else
	{
		wallAngle.x = -3000;
		wallAngle.y = -3000;
		// cout << "Angle too big " << endl;
	}

	px4->updateWallPos (wallAngle);
}

void ObjDetection::readTemplate()
{
	// tempImage = imread( "/home/tuofeichen/SLAM/MAV-Project/px4_ws/src/frontend/sp.jpg", IMREAD_GRAYSCALE);
  	tempImage = imread("/home/odroid/Images/rsz_tim.jpg",IMREAD_GRAYSCALE);
  	tempKeyPoints  	= dem->detect(tempImage);
	  tempDescriptors = dem->extract(tempImage,*tempKeyPoints);

	// cv::namedWindow("Signal Processing",CV_NORMAL);
	// cv::imshow("Signal Processing", tempImage);
	// cv::waitKey(0);
}
