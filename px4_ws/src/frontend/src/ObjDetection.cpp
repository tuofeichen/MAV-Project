#include "ObjDetection.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"


#define RANSAC 5
#define MIN_MATCHES 20
#define MAX_NORM 40

using namespace std;
using namespace cv;


int main()
{

}

ObjDetection::ObjDetection(IFeatures* aFDEM,RosHandler* aRos):
frame(new Frame) ,dem(aFDEM), px4(aRos)
{
	readTemplate(); // read template


}

void ObjDetection::processFrame(Frame newFrame)
{
	bool objDetected = 0;
	*frame = newFrame;

	// if (frame->getBadFrameFlag()!=1)
	objDetected = objectDetect();

	if (objDetected)
		checkObjAngle(frame->getDepth());
	else
		checkForWall(frame->getDepth());

	if(px4->getTakeoffFlag() && (!objDetected)) /// haven't taken off yet
		checkObstacles(frame->getDepth(),50,50,3000); // combine

}

bool  ObjDetection::objectDetect()
{
	std::vector< DMatch > forward_matches;
	std::vector< DMatch > backward_matches;
	std::vector< DMatch > matches;
	bool objFound = 0;

	if(!dem->match(*tempKeyPoints,*tempDescriptors, frame->getKeypoints(), frame->getDescriptors(), forward_matches))
	{
			return false;
	}

	else if (!dem->match(frame->getKeypoints(), frame->getDescriptors(), *tempKeyPoints,*tempDescriptors,backward_matches))
	{
			return false;
	}

	// cout << "forward match size " << forward_matches.size() << endl;
	// cout << "backward match size " << backward_matches.size() << endl << endl;

	std::vector<Point2f> obj;
	std::vector<Point2f> scene;

	cv::Mat H; // homography matrix

	for( int i = 0; i < backward_matches.size(); i++ ) // symmetry test
	{
// Get the keypoints from the good matches
		for (int j = 0; j < forward_matches.size();j++)
		{
			if (backward_matches.at(i).queryIdx == forward_matches.at(j).trainIdx)
				if(backward_matches.at(i).trainIdx == forward_matches.at(j).queryIdx)
				{
					matches.push_back(forward_matches.at(j));
					obj.push_back( (*tempKeyPoints)[ forward_matches[j].queryIdx ].pt );
					scene.push_back( frame->getKeypoints().at(forward_matches[j].trainIdx).pt );
					break;
				}
		}

	}


	// cout << matches.size() << endl;
	if (matches.size() < MIN_MATCHES)
		return false;


	Mat img_matches;
	drawMatches(tempImage, *tempKeyPoints, frame->getGray(), frame->getKeypoints(), matches, img_matches, Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::DEFAULT );
	H = findHomography( obj, scene, CV_RANSAC, RANSAC); // find homography
		//H = findHomography( obj, scene, CV_LMEDS);

	if(!H.empty())
	{
		Mat img_object = tempImage;
		Mat img_scene =  frame->getGray();
		std::vector<Point2f> obj_corners(4);

		obj_corners[0] = cvPoint(0,0);
		obj_corners[1] = cvPoint( img_object.cols, 0 );
		obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
		obj_corners[3] = cvPoint( 0, img_object.rows );
		std::vector<Point2f> scene_corners(4);

		perspectiveTransform(obj_corners, scene_corners, H);

		centroid = ( (scene_corners[0] + Point2f( img_object.cols, 0) ) + (scene_corners[1] + Point2f( img_object.cols, 0)) + (scene_corners[2] + Point2f( img_object.cols, 0)) + (scene_corners[3] + Point2f( img_object.cols, 0)) )/4;
		centroid2 = (scene_corners[0] + scene_corners[1] + scene_corners[2] + scene_corners[3])/4;

		if(centroid2.x > 0 && centroid2.x < frame->getGray().cols && centroid2.y > 0 && centroid2.y < frame->getGray().rows && norm(centroid2 - past_centroid2) < MAX_NORM)
		{

				// cout << "centroid is " << centroid2 << endl;
				// line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				// line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				// line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				// line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				// circle( img_matches, centroid, 32.0, Scalar( 0, 0, 255 ), 3, 8 );
				// circle( frame->getGray(), Point2f((int)centroid2.x,(int)centroid2.y), 32.0, Scalar( 255, 0, 0 ), 3, 8 );

				//circle( frame->getGray(), centroid2, 32.0, Scalar( 255, 0, 0 ), 3, 8 );

				// Get depth value if centroid position is in a valid position
				objPoint.z =-1;
				const cv::Mat& Depth = frame->getDepth();
				Point2f A = centroid2;
				const int X = (int)A.y;
				const int Y = (int)A.x ;


				float sum = 0;
				long int counter = 0;
				int test_counter = 0;

				for(int l=X-4;l<X+4;l++)
				{
					for(int k=Y-4;k<Y+4;k++)
						{
						if(Depth.at<uint16_t>(l,k) != 0 && l > 0 && l < frame->getGray().cols && k > 0 && k < frame->getGray().rows)
							{
							sum+=Depth.at<uint16_t>(l,k);
							counter++;
							}
						test_counter++;
						// cout << Depth.at<uint16_t>(l,k) << " ";
						}
					// cout << endl;
				}


				if(sum!=0)
				{

					cout << "x: " << X << endl;
					cout << "y: " << Y << endl;
					sum = sum/((float)counter);
					cout << "depth: " << sum << endl;
				}
				if(sum < 3500 && sum!=0)
				{
					// imshow( "ObjMatches", img_matches );
					objPoint.z = sum;
					objPoint.x = (int)centroid2.x;
					objPoint.y = (int)centroid2.y;
					px4->updateObjPos(objPoint);
					objFound = 1;
				}
		}

		past_centroid2 = centroid2;
		past_centroid = centroid;
	}
	return objFound;

	// cv::imshow("ObjDetect", frame->getRgb());
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
			int dist = depth.at<uint16_t>(160+j,120+i);
			if ((dist < safe_dist)&& (dist>0))
			{
				obstCnt++;
				totalDist += dist;
				minDist = std::min(minDist,dist);
			}
		}
	}

	obstacleDistance.x = (float) obstCnt; // some threshold (what is a rigorous check)
	obstacleDistance.y = (float) minDist;
	obstacleDistance.z = (float) totalDist / obstCnt; // average distance

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
	float alpha, betha;
	float angle_per_pixel_rad = (58./320.)*M_PI/180.;
	bool valid_position = true;

	float distance = 0;
	int   distance_counter = 0;


	float FOV = (pixel_number_right - pixel_number_left)*angle_per_pixel_rad;


	for(int k = e; k < 100; k++)
	{
		pixel_number_left = 160-e;
		pixel_number_right = 160+e;

		for (int i=pixel_number_right;i<pixel_number_right + square_size;i++)
		{
    	 	for (int j=height; j< height+square_size;j++)
				{
					if(Depth.at<uint16_t>(j,i)!=0)
					{
						r = r + Depth.at<uint16_t>(j,i);
        	      		r_counter = r_counter + 1;
					}
				}
		}
 		r=r/(float)r_counter;

		for (int i=pixel_number_left;i<pixel_number_left + square_size;i++)
		{
     		for (int j=height; j< height+square_size;j++)
				{
					if(Depth.at<uint16_t>(j,i)!=0)
					{
						l = l + Depth.at<uint16_t>(j,i);
              			l_counter = l_counter + 1;
					}
				}
		}
 		l=l/(float)l_counter;

		//float max=1;
		//	if(r>=l) max = r;
		//	else max = l;

		//cout << "max: " << max(r,l) << endl;


		if(fabs(r-Depth.at<uint16_t>(height,pixel_number_right)) < 15 && fabs(l-Depth.at<uint16_t>(height,pixel_number_left)) < 15)
		{
			valid_position = true;
			break;

		}

	}


	if(valid_position && fabs(r-l)/max(r,l) < 0.2 )
	{
		float D = sqrt(r*r + l*l - 2*l*r*cos(FOV));


		if(l > r)
		{
 	 		 alpha = asin(r/D*sin(FOV));
   			 objAngle.x = alpha + FOV/2 - M_PI/2;
		}
		else
		{
    		betha = asin(l/D*sin(FOV));
    		objAngle.x = M_PI/2 - FOV/2 - betha;
		}

		if(l < 3000 && r < 3000 && r!=0 && l!=0)
		{

			objAngle.y = objAngle.x*180/M_PI; // in degrees
		}
		else
		{
			objAngle.x = -5000;
			objAngle.y = -5000;
		}

	    // cout << "objAngle(in degrees): " << objAngle.y << endl;
		// cout << "objAngle(in rad): " << objAngle.x << endl;
	}
	else
	{
		objAngle.x = -5000;
		objAngle.y = -5000;
	//	cout << "Invalid position" << endl;
	}


	px4->updateWallPos (objAngle);
}
void ObjDetection::checkForWall(cv::Mat Depth) // these functions needs clean up tbh
{
	int e = 20;
	int square_size = 5;
	int pixel_number_left = 160-e;
	int pixel_number_right = 160+e;
	float r=0;
	float l=0;
	int r_counter=0;
	int l_counter=0;
	float alpha, betha;
	float angle_per_pixel_rad = (58./320.)*M_PI/180.;
	bool valid_position = true;

	float distance = 0;
	int   distance_counter = 0;


	float FOV = (pixel_number_right - pixel_number_left)*angle_per_pixel_rad;


	for(int k = e; k < 100; k++)
	{
		pixel_number_left = 160-e;
		pixel_number_right = 160+e;

		for (int i=pixel_number_right;i<pixel_number_right + square_size;i++)
		{
    	 	for (int j=180; j< 180+square_size;j++)
				{
					if(Depth.at<uint16_t>(j,i)!=0)
					{
						r = r + Depth.at<uint16_t>(j,i);
        	      		r_counter = r_counter + 1;
					}
				}
		}
 		r=r/(float)r_counter;

		for (int i=pixel_number_left;i<pixel_number_left + square_size;i++)
		{
     		for (int j=180; j< 180+square_size;j++)
				{
					if(Depth.at<uint16_t>(j,i)!=0)
					{
						l = l + Depth.at<uint16_t>(j,i);
              			l_counter = l_counter + 1;
					}
				}
		}
 		l=l/(float)l_counter;

		//float max=1;
		//	if(r>=l) max = r;
		//	else max = l;

		//cout << "max: " << max(r,l) << endl;


		if(fabs(r-Depth.at<uint16_t>(180,pixel_number_right)) < 15 && fabs(l-Depth.at<uint16_t>(180,pixel_number_left)) < 15)
		{
			valid_position = true;
			break;

		}

	}


	if(valid_position && fabs(r-l)/max(r,l) < 0.2 )
	{
		float D = sqrt(r*r + l*l - 2*l*r*cos(FOV));


		if(l > r)
		{
 	 		 alpha = asin(r/D*sin(FOV));
   			 wallAngle.x = alpha + FOV/2 - M_PI/2;
		}
		else
		{
    		betha = asin(l/D*sin(FOV));
    		wallAngle.x = M_PI/2 - FOV/2 - betha;
		}

		if(l < 3000 && r < 3000 && r!=0 && l!=0)
		{

			wallAngle.y = wallAngle.x*180/M_PI; // in degrees
		}
		else
		{
			wallAngle.x = -5000;
			wallAngle.y = -5000;
		}

	    // cout << "wallAngle(in degrees): " << wallAngle.y << endl;
		// cout << "wallAngle(in rad): " << wallAngle.x << endl;
	}
	else
	{
		wallAngle.x = -5000;
		wallAngle.y = -5000;
	//	cout << "Invalid position" << endl;
	}


	for (int i=155;i<165;i++)
	{
    	for (int j=115; j < 125;j++)
		{
			if(Depth.at<uint16_t>(j,i)!=0)
			{
						distance = distance + Depth.at<uint16_t>(j,i);
        	      		distance_counter = distance_counter + 1;
			}
		}
	}

	wallAngle. z = distance / (float) distance_counter; // how far from the wall;

	px4->updateWallPos (wallAngle);
}

void ObjDetection::readTemplate()
{
	tempImage = imread( "/home/odroid/painting.jpg", IMREAD_GRAYSCALE);
	tempKeyPoints  	= dem->detect(tempImage);
	tempDescriptors = dem->extract(tempImage,*tempKeyPoints);

	// cv::namedWindow("scientist",CV_NORMAL);
	// cv::imshow("scientist", tempImage);
	// cv::waitKey(0);
}
