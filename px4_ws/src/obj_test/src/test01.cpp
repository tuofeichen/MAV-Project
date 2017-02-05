#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "SURF.h"
#include "SIFT.h"
#include "OrbDetSurfDesc.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "Frame.h"
#include "AsusProLiveOpenNI2.h"
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include <fstream>

using namespace cv;
using namespace std;
using namespace SLAM;

#define SURF_RADIUS 0.45f
//#define SURF_RADIUS 0.25f
#define MATCHES_SIZE 10
#define RANSAC 5
#define NORM 40
#define PI 3.14159
 

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//// get template picture						////////
//// detect picture feoatures					////////	
//// extract picture features					////////
////										    ////////
//// while (cond){								////////
//// 		get SCENE_IMAGE						////////
////		detect SCENE_IMAGE features			////////
////		extract SCENE_IMAGE features		////////
////											////////
////		match								////////
////										    ////////
////		show match features					////////
////             }								////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

static std::ofstream logDepth; 
static std::ofstream logImage; 
void WallFind(cv::Mat depth);

geometry_msgs::Point wallAngle;


std_msgs::Float64 distance_to_wall;
int frame_counter = 1;
void CheckForWall(cv::Mat depth);
void Distance(cv::Mat depth);

int main(int argc, char* argv[])
{

	bool camera = false;
	bool camera_depth = true;

	logDepth.open("/home/felipe/catkin_ws/src/obj_test/Depth.csv", std::ofstream::out | std::ofstream::trunc);
	logImage.open("/home/felipe/catkin_ws/src/obj_test/Image.csv", std::ofstream::out | std::ofstream::trunc);	
		
	//logDepth << "Depth Map for Feature Detection" << endl; 
	//logImage << "Image map " << endl;
	
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
	cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
	//cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
	//cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
	
	ros::init(argc, argv, "FindObject");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	ros::Publisher FindObjectPub = nh.advertise<geometry_msgs::Point>("/objDetect/obj_pose", 1000);
	ros::Publisher wall_pub = nh.advertise<geometry_msgs::Point>("/objDetect/wall_pose", 1000);
	ros::Publisher distance_to_wall_pub = nh.advertise<std_msgs::Float64>("DistanceToWall", 1000);
	
	geometry_msgs::Point PublishPoint;

	TEMPLATE = imread( "/home/odroid/painting.jpg", IMREAD_GRAYSCALE );
	
	
	if(camera_depth) 
		AsusProLiveOpenNI2::start();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if(camera_depth)
	{
		for(int i = 0; i <5 ; ++i)
		{
			if (!AsusProLiveOpenNI2::grab(frame))
			{	
				cout << " init " << endl; 	
				boost::this_thread::sleep(boost::posix_time::milliseconds(10));
				i--;
			}
		} 
			
	}
	cout << "init finished" << endl;
		


// Get TEMPLATE parameters
		TEMPLATE_KEYPOINTS  = feature_detector_obj.detect(TEMPLATE);
		TEMPLATE_DESCRIPTOR = feature_detector_obj.extract(TEMPLATE,*TEMPLATE_KEYPOINTS);
///////////////////////////////////////////////////////////////////////////////////////////////

	

	//for(;;)
while(ros::ok() )
	{	
// Pre-starting the RGB-D camera
	logImage << "Frame: " << frame_counter << endl;
	logDepth << "Frame: " << frame_counter << endl;
	frame_counter++;
	
	
		if(camera_depth)
		{
			if (!AsusProLiveOpenNI2::grab(frame))
			{
				boost::this_thread::sleep(boost::posix_time::milliseconds(10));
				cout << " bad frame" << endl; 	
				continue;
			}	 
		}
	
/////////////////////////////////////////////////////////////////////////////////////////////



// Grab frame from camera		
		if(camera_depth) SCENE_IMAGE = frame.getGray();
		if(camera) cap.read(SCENE_IMAGE);
	
		//cout << "1" << endl;

		//imshow("test",SCENE_IMAGE);
/////////////////////////////////////////////////////////////////////////////////////////////

	//cout <<



// Get SCENE_IMAGE parameters
		SCENE_IMAGE_KEYPOINTS  = feature_detector_obj.detect(SCENE_IMAGE);
		SCENE_IMAGE_DESCRIPTOR = feature_detector_obj.extract(SCENE_IMAGE,*SCENE_IMAGE_KEYPOINTS);
///////////////////////////////////////////////////////////////////////////////////////////////
		//cout << "2" << endl;


		(*SCENE_IMAGE_DESCRIPTOR).convertTo(*SCENE_IMAGE_DESCRIPTOR,CV_32F);
	 	(*TEMPLATE_DESCRIPTOR).convertTo(*TEMPLATE_DESCRIPTOR,CV_32F);
	
 if(((*SCENE_IMAGE_KEYPOINTS).empty() ) || (*SCENE_IMAGE_DESCRIPTOR).empty() || (*TEMPLATE_DESCRIPTOR).empty() )
 {
	 
	 //cout <<  (*SCENE_IMAGE_KEYPOINTS).size() << " " << (*SCENE_IMAGE_DESCRIPTOR).size() << " " << (*TEMPLATE_DESCRIPTOR).size() << endl;
	 //cout << "3" << endl;
	 // cout << " size of the descriptor matrix " << (*SCENE_IMAGE_DESCRIPTOR).size? << endl; 
	 // cout << " bad frame " << endl; 
 }
else
 {
	//cout << "good frame " << endl;
	//cout << "4" << endl;
		// cout << " normal frame" << endl; 	 
// Matching features from TEMPLATE with SCENE_IMAGE
	
	//cout << (*TEMPLATE_KEYPOINTS).size() << ", " << (*TEMPLATE_DESCRIPTOR).size() << endl;
	//cout << (*TEMPLATE_KEYPOINTS).size() << ", " << (*TEMPLATE_DESCRIPTOR).size() << ", " <<  (*SCENE_IMAGE_KEYPOINTS).size() << ", " << (*SCENE_IMAGE_DESCRIPTOR).size() << endl;
	//cout << TEMPLATE_KEYPOINTS_INDICES.size() << ", " << SCENE_KEYPOINTS_INDICES.size() << endl;
	
	
	//cout << (*TEMPLATE_KEYPOINTS).size() << ", " << (*TEMPLATE_DESCRIPTOR).size() << ", " << 
		//cout << "match: " << endl;
		feature_detector_obj.match(*TEMPLATE_KEYPOINTS, *TEMPLATE_DESCRIPTOR, *SCENE_IMAGE_KEYPOINTS, *SCENE_IMAGE_DESCRIPTOR, TEMPLATE_KEYPOINTS_INDICES, SCENE_KEYPOINTS_INDICES);
///////////////////////////////////////////////////////////////////////////////////////////////
	//cout << "uoi, e aqui?" << endl;



// Create vector so we can use the drawMatches function
		vector<cv::DMatch> matches (TEMPLATE_KEYPOINTS_INDICES.size());

		//cout << "aqui1" << endl;
		for(int i=0; i<TEMPLATE_KEYPOINTS_INDICES.size(),i<SCENE_KEYPOINTS_INDICES.size(); i++)
			{
			cv::DMatch L;
		//	cout << "aqui2" << endl;

			L.queryIdx = TEMPLATE_KEYPOINTS_INDICES[i];
		//	cout<<"aqui3" << endl;
			L.trainIdx = SCENE_KEYPOINTS_INDICES[i];
		//	cout <<"aqui4"<<endl;
			matches[i] = L;
		//	cout <<"aqui5"<<endl;

			Point2f point_TEMPLATE = (*TEMPLATE_KEYPOINTS)[i].pt;
		//	cout<<"aqui6"<<endl;
			Point2f point_SCENE_IMAGE = (*SCENE_IMAGE_KEYPOINTS)[i].pt;
		//	cout<<"aqui7"<<endl;
			}

	
	    
		Mat img_matches;
	    
	   //cout << "5" << endl;
		drawMatches( TEMPLATE, *TEMPLATE_KEYPOINTS, SCENE_IMAGE, *SCENE_IMAGE_KEYPOINTS, matches, img_matches, Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::DEFAULT );
///////////////////////////////////////////////////////////////////////////////////////////////
		//cout << "6" << endl;



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
			
			
			//double distance1 = sqrt(  pow((centroid2.x - obj_corners[0].x ),2) + pow((centroid2.y - obj_corners[0].y ),2)  );
			//double distance2 = sqrt(  pow((centroid2.x - obj_corners[1].x ),2) + pow((centroid2.y - obj_corners[1].y ),2)  );
			//double distance3 = sqrt(  pow((centroid2.x - obj_corners[2].x ),2) + pow((centroid2.y - obj_corners[2].y ),2)  );
			//double distance4 = sqrt(  pow((centroid2.x - obj_corners[3].x ),2) + pow((centroid2.y - obj_corners[3].y ),2)  );
			
			
			
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
						//cout << Depth.at<uint16_t>(l,k) << " ";
						}
					//cout << endl;
				}
				
				//
				//			
 
		
				//cout << "x: " << X << endl;
				//cout << "y: " << Y << endl;

				if(sum!=0) sum = sum/((float)counter);
				//cout << "depth: " << sum << endl; 
					

					// save Depth variable in a excel document
					//logDepth << "centroid position," << centroid2.x << "," <<centroid2.y << ",sum," << sum << endl;
					//logDepth << frame.getDepth() << endl << endl;  
						
					//cout << "saved" << endl;
					//return 0;
					
				//cout << "distance: " << sum << " counter " << counter << endl; 
				//cout << "x: " << centroid2.x << " y: " << centroid2.y << endl; 
				//counter = 0;
				//cout << "test counter: " << test_counter << endl;
				//cout << "//" << endl;
					//cout << Depth << endl;
					//cout << "//" << endl;
				
				//if(sum < 2000 && sum!=0) 
				PublishPoint.z = sum;
				}
				
// Publishing the object position and its depth
				
				
				PublishPoint.x = (int)centroid2.x;
				PublishPoint.y = (int)centroid2.y;
				
				
				//double area = (x1*y2 - y1*x2) + (x2*y3 - y2*x3) + (x3*y4 - y3*x4) + (x4*y1 - y4*x1);
				
				//cout << scene_corners[0] << ", " << scene_corners[1] << ", " << scene_corners[2] << ", " << scene_corners[3] << endl;
				
				double area = ((scene_corners[0].x * scene_corners[1].y - scene_corners[0].y * scene_corners[1].x) + (scene_corners[1].x * scene_corners[2].y - scene_corners[1].y * scene_corners[2].x) + (scene_corners[2].x * scene_corners[3].y - scene_corners[2].y * scene_corners[3].x) + (scene_corners[3].x * scene_corners[0].y - scene_corners[3].y * scene_corners[0].x))/2;
				//cout << "area: " << area << endl;
				//double area = (scene_corners[0].x * scene_corners[1].y - scene_corners[0].y*
				if(PublishPoint.z == 0)
				{
					// 5429*x^(-0.51) , input: area, output: distance
					//PublishPoint.z = 5429*pow(area,-0.51);	
					//cout << "hi" << endl;
					//PublishPoint.z = 55.769597 + -0.00118966873*area 8.5577208e-9*area;
					//PublishPoint.z = (55.769597 -0.00118966873*area + (8.5577208E-9)*area*area)*10; 
					if(PublishPoint.z = (48.9599122 - 6.61309228E-4*area + (2.345900306E-9)*area*area)!=0)
							PublishPoint.z = (48.9599122 - 6.61309228E-4*area + (2.345900306E-9)*area*area)*10;


				}
				
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
	//imshow( "Matches", img_matches );
	
	
	char key = cvWaitKey(10);
			if (key == 27) // ESC
            break;
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
	
    
	
	if(camera_depth){
	const cv::Mat& depth = frame.getDepth();
	//WallFind(depth);
	
	//geometry_msgs::Point Depth;
	//Depth.x  = depth.at<uint16_t>((int)(depth.rows/2),(int)(depth.cols/2)); // center
    //Depth.y  = depth.at<uint16_t>((int)(depth.rows/2),(int)(depth.cols));   // right most
	//Depth.z  = depth.at<uint16_t>((int)(depth.rows/2),1);					// left most 
	//cout << "depth: " << Depth << endl;
	circle( SCENE_IMAGE, Point2f((int)(depth.cols/2),(int)(depth.rows/2)), 5.0, Scalar( 255, 0, 255 ), 3, 8 );
	circle( SCENE_IMAGE, Point2f((int)(depth.cols-1),(int)(depth.rows/2)), 5.0, Scalar( 255, 0, 255 ), 3, 8 );	
	circle( SCENE_IMAGE, Point2f(1,(int)(depth.rows/2)), 5.0, Scalar( 255, 0, 255 ), 3, 8 );	
	//imshow("test",SCENE_IMAGE);	
		
	CheckForWall(depth);	
	wall_pub.publish(wallAngle);
	Distance(depth);
	distance_to_wall_pub.publish(distance_to_wall);
	//wall_pub.publish(depth);
		//
		
	Mat show;
	depth.convertTo( show, CV_8UC1, 0.05f);
	//imshow( "depth map", show);}
////////////////////////////////////////////////////////////////////////////////////////////// 

	char key = cvWaitKey(10);
			if (key == 27) // ESC
            break;
	
	//logDepth << frame.getDepth() << endl << endl;
	//logImage << SCENE_IMAGE << endl << endl;
	
	loop_rate.sleep();	
	ros::spinOnce();
	
	}
	
	
}

}


void Distance(cv::Mat Depth)
{
		float distance = 0;
		int distance_counter = 0;
	

			
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
	
 		distance=distance/(float)distance_counter;
	
		distance_to_wall.data = distance;
		
}



void CheckForWall(cv::Mat Depth)
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
	// geometry_msgs::Point wallAngle;

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
}

