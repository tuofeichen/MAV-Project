#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "SURF.h"
//#include "SIFT.h"
//#include "OrbDetSurfDesc.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "Frame.h"
#include "AsusProLiveOpenNI2.h"
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <fstream>

using namespace cv;
using namespace std;
using namespace SLAM;

#define SURF_RADIUS 0.7f
//#define SURF_RADIUS 0.25f
#define MATCHES_SIZE 15
#define RANSAC 5
#define NORM 40
 

////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//// get template picture						////////
//// detect picture feoatures					////////	
//// extract picture features					////////
////										   ////////
//// while (cond){								////////
//// 		get SCENE_IMAGE						////////
////		detect SCENE_IMAGE features			////////
////		extract SCENE_IMAGE features		////////
////											////////
////		match								////////
////										    ////////
////		show match features					////////
////             }								////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

static std::ofstream logDepth; 



int main(int argc, char* argv[])
{

	bool camera = false;
	bool camera_depth = true;

	logDepth.open("/home/felipe/catkin_ws/src/obj_test/Depth.csv", std::ofstream::out | std::ofstream::trunc);
		
		
	logDepth << "Depth Map for Feature Detection" << endl; 
	
	
// Initialize variables
	SURF feature_detector_obj(SURF_RADIUS, 50, 600);
	//SIFT feature_detector_obj(SURF_RADIUS, 50, 600);
	//OrbDetSurfDesc feature_detector_obj(SURF_RADIUS, 50,600, 600);

	Mat TEMPLATE;
	Mat SCENE_IMAGE;
	vector<int> TEMPLATE_KEYPOINTS_INDICES;
	vector<int> SCENE_KEYPOINTS_INDICES;
	Frame frame;
	Mat gray;
	boost::shared_ptr<std::vector<cv::KeyPoint> > TEMPLATE_KEYPOINTS; 
	boost::shared_ptr<cv::Mat> TEMPLATE_DESCRIPTOR;
	boost::shared_ptr<std::vector<cv::KeyPoint> > SCENE_IMAGE_KEYPOINTS;
	boost::shared_ptr<cv::Mat> SCENE_IMAGE_DESCRIPTOR;
	Point2f past_centroid;
	Point2f past_centroid2;
	cv::VideoCapture cap(0);
	ros::init(argc, argv, "FindObjectPub");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	ros::Publisher FindObjectPub = nh.advertise<geometry_msgs::Point>("FindObjectSub", 1000);
	geometry_msgs::Point PublishPoint;
	//TEMPLATE = imread( "/home/felipe/catkin_ws/src/obj_test/book_cover_1.jpg", IMREAD_GRAYSCALE );
	//TEMPLATE = imread( "/home/felipe/catkin_ws/src/obj_test/witch.jpeg", IMREAD_GRAYSCALE );
	//TEMPLATE = imread( "/home/felipe/catkin_ws/src/obj_test/coke.jpg", IMREAD_GRAYSCALE );
	//TEMPLATE = imread( "/home/felipe/catkin_ws/src/obj_test/tide.jpg", IMREAD_GRAYSCALE );
	//TEMPLATE = imread( "/home/felipe/catkin_ws/src/obj_test/tide_box.jpg", IMREAD_GRAYSCALE );	
	//TEMPLATE = imread( "/home/felipe/catkin_ws/src/obj_test/box.jpg", IMREAD_GRAYSCALE );
	TEMPLATE = imread( "/home/felipe/catkin_ws/src/obj_test/book.jpg", IMREAD_GRAYSCALE );
	//TEMPLATE = imread( "/home/felipe/catkin_ws/src/obj_test/logo.png", IMREAD_GRAYSCALE );
	
	if(camera_depth) AsusProLiveOpenNI2::start();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if(camera_depth){
		for(int i = 0; i <5 ; ++i){
			if (!AsusProLiveOpenNI2::grab(frame))
			{	
				cout << " init " << endl; 	
				boost::this_thread::sleep(boost::posix_time::milliseconds(10));
				i--;
			}
								   } 
			
					}
		


// Get TEMPLATE parameters
		TEMPLATE_KEYPOINTS  = feature_detector_obj.detect(TEMPLATE);
		TEMPLATE_DESCRIPTOR = feature_detector_obj.extract(TEMPLATE,*TEMPLATE_KEYPOINTS);
///////////////////////////////////////////////////////////////////////////////////////////////

	

	//for(;;)
while(ros::ok() )
	{	
// Pre-starting the RGB-D camera
		if(camera_depth){
			if (!AsusProLiveOpenNI2::grab(frame)){
				boost::this_thread::sleep(boost::posix_time::milliseconds(10));
				cout << " bad frame" << endl; 	
				continue;
												}	 
						}
/////////////////////////////////////////////////////////////////////////////////////////////



// Grab frame from camera		
		if(camera_depth) SCENE_IMAGE = frame.getGray();
		if(camera) cap.read(SCENE_IMAGE);

		//imshow("test",SCENE_IMAGE);
/////////////////////////////////////////////////////////////////////////////////////////////





// Get SCENE_IMAGE parameters
		SCENE_IMAGE_KEYPOINTS  = feature_detector_obj.detect(SCENE_IMAGE);
		SCENE_IMAGE_DESCRIPTOR = feature_detector_obj.extract(SCENE_IMAGE,*SCENE_IMAGE_KEYPOINTS);
///////////////////////////////////////////////////////////////////////////////////////////////




// Matching features from TEMPLATE with SCENE_IMAGE
		feature_detector_obj.match(*TEMPLATE_KEYPOINTS, *TEMPLATE_DESCRIPTOR, *SCENE_IMAGE_KEYPOINTS, *SCENE_IMAGE_DESCRIPTOR, TEMPLATE_KEYPOINTS_INDICES, SCENE_KEYPOINTS_INDICES);
///////////////////////////////////////////////////////////////////////////////////////////////


// Create vector so we can use the drawMatches function
		vector<cv::DMatch> matches (TEMPLATE_KEYPOINTS_INDICES.size());


		for(int i=0; i<TEMPLATE_KEYPOINTS_INDICES.size(),i<SCENE_KEYPOINTS_INDICES.size(); i++)
			{
			cv::DMatch L;

			L.queryIdx = TEMPLATE_KEYPOINTS_INDICES[i];
			L.trainIdx = SCENE_KEYPOINTS_INDICES[i];
			matches[i] = L;

			Point2f point_TEMPLATE = (*TEMPLATE_KEYPOINTS)[i].pt;
			Point2f point_SCENE_IMAGE = (*SCENE_IMAGE_KEYPOINTS)[i].pt;
			}

		Mat img_matches;
		drawMatches( TEMPLATE, *TEMPLATE_KEYPOINTS, SCENE_IMAGE, *SCENE_IMAGE_KEYPOINTS, matches, img_matches, Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::DEFAULT );
///////////////////////////////////////////////////////////////////////////////////////////////




// Finding Object in the SCENE
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for( int i = 0; i < matches.size(); i++ )
		{
// Get the keypoints from the good matches
			obj.push_back( (*TEMPLATE_KEYPOINTS)[ matches[i].queryIdx ].pt );
			scene.push_back( (*SCENE_IMAGE_KEYPOINTS)[ matches[i].trainIdx ].pt );
		}
////////////////////////////////////////////////////////////////////////////
		Mat H;
		if(!obj.empty() && !scene.empty())
		{
			H = findHomography( obj, scene, CV_RANSAC, RANSAC); 
		//H = findHomography( obj, scene, CV_LMEDS);			
		}

		if(matches.size() > MATCHES_SIZE && !H.empty())
		{		
// Build square arround matching area
			Mat img_object = TEMPLATE;
			Mat img_scene = SCENE_IMAGE;

			std::vector<Point2f> obj_corners(4);
			obj_corners[0] = cvPoint(0,0);
			obj_corners[1] = cvPoint( img_object.cols, 0 );
			obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); 
			obj_corners[3] = cvPoint( 0, img_object.rows );
			std::vector<Point2f> scene_corners(4);

			perspectiveTransform( obj_corners, scene_corners, H);

// Calculate the center of the square			
			Point2f centroid = ( (scene_corners[0] + Point2f( img_object.cols, 0) ) + (scene_corners[1] + Point2f( img_object.cols, 0)) + (scene_corners[2] + Point2f( img_object.cols, 0)) + (scene_corners[3] + Point2f( img_object.cols, 0)) )/4;
			Point2f centroid2 = (scene_corners[0] + scene_corners[1] + scene_corners[2] + scene_corners[3])/4;

// Draw square in the output image 
					// if the centroid has a valid number and if the distance between the current centroid and the previous centroid
					// is not too high
			if(centroid2.x > 0 && centroid2.x < SCENE_IMAGE.cols && centroid2.y > 0 && centroid2.y < SCENE_IMAGE.rows && norm(centroid2 - past_centroid2) < NORM)
			{ 
				line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
				line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				circle( img_matches, centroid, 32.0, Scalar( 0, 0, 255 ), 3, 8 );

				circle( SCENE_IMAGE, Point2f((int)centroid2.x,(int)centroid2.y), 32.0, Scalar( 255, 0, 0 ), 3, 8 );
				//circle( SCENE_IMAGE, centroid2, 32.0, Scalar( 255, 0, 0 ), 3, 8 );
				
// Get depth value if centroid position is in a valid position
				PublishPoint.z =-1;
				if(camera_depth)
				{
				const cv::Mat& Depth = frame.getDepth();
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
						if(Depth.at<uint16_t>(l,k) != 0 && l > 0 && l < SCENE_IMAGE.cols && k > 0 && k < SCENE_IMAGE.rows)
							{
							sum+=Depth.at<uint16_t>(l,k);
							counter++;
							}
						test_counter++;
						cout << Depth.at<uint16_t>(l,k) << " ";
						}
					cout << endl;
				}
				
				//
				//			
 
		
				cout << "x: " << X << endl;
				cout << "y: " << Y << endl;

				if(sum!=0) sum = sum/((float)counter);
				cout << "depth: " << sum << endl; 
					
				if(true)
				{
					// save Depth variable in a excel document
					//logDepth << "centroid position," << centroid2.x << "," <<centroid2.y << ",sum," << sum << endl;
					//logDepth << frame.getDepth() << endl << endl;  
						
					//cout << "saved" << endl;
					//return 0;
				}
					
				//cout << "distance: " << sum << " counter " << counter << endl; 
				//cout << "x: " << centroid2.x << " y: " << centroid2.y << endl; 
				//counter = 0;
				//cout << "test counter: " << test_counter << endl;
				//cout << "//" << endl;
					//cout << Depth << endl;
					//cout << "//" << endl;
				
				if(sum < 2000 && sum!=0) PublishPoint.z = sum;
				}
				
// Publishing the object position and its depth
				PublishPoint.x = (int)centroid2.x;
				PublishPoint.y = (int)centroid2.y;
				
				FindObjectPub.publish(PublishPoint);
				
				

			}
		
			if(centroid2.x <= 0 && centroid2.x >= SCENE_IMAGE.cols && centroid2.y <= 0 && centroid2.y >= SCENE_IMAGE.rows && norm(centroid2 - past_centroid2) >= NORM)
			{
			cout << "centroid error: " << centroid2 << endl;
			centroid=past_centroid;		
			circle( img_matches, centroid, 32.0, Scalar( 0, 0, 255 ), 3, 8 );
				
			}

			//cout << "centroid: " << centroid2 << endl;
			//cout << "past centroid: " <<past_centroid << endl;
			//cout << "norm: " << norm(centroid2 - past_centroid2) << endl;
			past_centroid2 = centroid2;
			past_centroid = centroid;

////////////////////////////////////////////////////////////////////////////////////////////
	}

	/*
	float sum = 0;
	long int counter = 0;
	
					
				for(int l=0;l<depth.rows;l++)
				{
					for(int k=0;k<depth.cols;k++)
						{
						if(depth.at<uint16_t>(l,k) != 0)
							{
							sum+=depth.at<uint16_t>(l,k);
							counter++;
							}
						//test_counter++;
						//cout << Depth.at<uint16_t>(l,k) << " ";
						}
					//cout << endl;
				}
	
				if(sum!=0) sum = sum/((float)counter);
				cout << "depth: " << sum << endl;*/

		
		
// Output matching picture
	imshow( "Matches", img_matches );
    imshow("test",SCENE_IMAGE);
	
	if(camera_depth){
	const cv::Mat& depth = frame.getDepth();
	Mat show;
	depth.convertTo( show, CV_8UC1, 0.05f);
	imshow( "depth map", show);}
////////////////////////////////////////////////////////////////////////////////////////////// 

	char key = cvWaitKey(10);
			if (key == 27) // ESC
            break;
	}

}



