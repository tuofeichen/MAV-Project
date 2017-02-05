//SAVE 07/13/2016


#include <iostream>
#include <opencv2/core.hpp>
//#include <highgui.h>
#include <opencv2/highgui.hpp>
#include "SURF.h"
//#include <cv.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "AsusProLive.h"
//#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
using namespace SLAM;

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//// get template picture			////////
//// detect picture features			////////	
//// extract picture features			////////
////						////////
//// while (cond){				////////
//// 		get SCENE_IMAGE			////////
////		detect SCENE_IMAGE features	////////
////		extract SCENE_IMAGE features	////////
////						////////
////		match				////////
////						////////
////		show match features		////////
////             }				////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

int main(int argc, char* argv[]){

cout << "comeca2" << endl;


// Initialize
SURF feature_detector_obj(0.6f, 50, 600);
//Mat TEMPLATE = imread( argv[1], IMREAD_GRAYSCALE ); 
//cv::VideoCapture cap(argv[2]);
Mat TEMPLATE = imread( "/home/felipe/catkin_ws/src/obj_test/book_cover_1.jpg", IMREAD_GRAYSCALE );
//Mat TEMPLATE = imread( "book_cover_1.jpg",1);
//imwrite("rola.jpg",TEMPLATE);
cv::VideoCapture cap("/home/felipe/catkin_ws/src/obj_test/video1.MP4");
//Mat TEMPLATE;
cout << "comeca2" << endl;
//cv::VideoCapture cap;
cout << "comeca3" << endl;
// 
Mat SCENE_IMAGE;
vector<int> TEMPLATE_KEYPOINTS_INDICES;
vector<int> SCENE_KEYPOINTS_INDICES;
//boost::shared_ptr<cv::Mat> grayImage;
//boost::shared_ptr<cv::Mat> rgbImage;
//boost::shared_ptr<cv::Mat> depthImage;
//boost::shared_ptr<double> timeStamp;
//AsusProLive::start();
//VideoCapture cap("video1.mp4");
cout << "comeca4" << endl;
cout << TEMPLATE.size() << endl;

//cv::namedWindow("test");
//cv::startWindowThread();
//imshow( "test",  TEMPLATE);
//waitKey(0);

for(;;){
//AsusProLive::grab(*rgbImage, *grayImage, *depthImage, *timeStamp);
//SCENE_IMAGE = TEMPLATE;
cap.read(SCENE_IMAGE);
cout << "comeca5" << endl;
//SCENE_IMAGE = imread( SCENE_IMAGE, IMREAD_GRAYSCALE ); 

// Get template TEMPLATE parameters : TEMPLATE
boost::shared_ptr<std::vector<cv::KeyPoint> > TEMPLATE_KEYPOINTS; 
boost::shared_ptr<cv::Mat> TEMPLATE_DESCRIPTOR;


TEMPLATE_KEYPOINTS  = feature_detector_obj.detect(TEMPLATE);
cout << "comeca6" << endl;
TEMPLATE_DESCRIPTOR = feature_detector_obj.extract(TEMPLATE,*TEMPLATE_KEYPOINTS);


// Get SCENE_IMAGE: SCENE_IMAGE
boost::shared_ptr<std::vector<cv::KeyPoint> > SCENE_IMAGE_KEYPOINTS;
boost::shared_ptr<cv::Mat> SCENE_IMAGE_DESCRIPTOR;
SCENE_IMAGE_KEYPOINTS  = feature_detector_obj.detect(SCENE_IMAGE);
SCENE_IMAGE_DESCRIPTOR = feature_detector_obj.extract(SCENE_IMAGE,*SCENE_IMAGE_KEYPOINTS);

cout << "comeca7" << endl;
// Matching features
feature_detector_obj.match(*TEMPLATE_KEYPOINTS, *TEMPLATE_DESCRIPTOR, *SCENE_IMAGE_KEYPOINTS, *SCENE_IMAGE_DESCRIPTOR, TEMPLATE_KEYPOINTS_INDICES, SCENE_KEYPOINTS_INDICES);
//
cout << "comeca8" << endl;

//////////////////////// just for showing
vector<cv::DMatch> matches (TEMPLATE_KEYPOINTS_INDICES.size());
cout << TEMPLATE_KEYPOINTS_INDICES.size() << endl;

for(int i=0; i<TEMPLATE_KEYPOINTS_INDICES.size(),i<SCENE_KEYPOINTS_INDICES.size(); i++){
	cv::DMatch L;

	L.queryIdx = TEMPLATE_KEYPOINTS_INDICES[i];
	L.trainIdx = SCENE_KEYPOINTS_INDICES[i];
	matches[i] = L;

	Point2f point_TEMPLATE = (*TEMPLATE_KEYPOINTS)[i].pt;
	Point2f point_SCENE_IMAGE = (*SCENE_IMAGE_KEYPOINTS)[i].pt;

	//cout << i << endl;
}


Mat img_matches;
drawMatches( TEMPLATE, *TEMPLATE_KEYPOINTS, SCENE_IMAGE, *SCENE_IMAGE_KEYPOINTS, matches, img_matches, Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::DEFAULT );


std::vector<Point2f> obj;
std::vector<Point2f> scene;

  for( int i = 0; i < matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( (*TEMPLATE_KEYPOINTS)[ matches[i].queryIdx ].pt );
    scene.push_back( (*SCENE_IMAGE_KEYPOINTS)[ matches[i].trainIdx ].pt );
  }
////////////////////////////////////////////////////////////////////////////
cout << "here" << endl;
if(matches.size() > 0){
/////////////// drawing square
Mat H = findHomography( obj, scene, CV_RANSAC );
Mat img_object = TEMPLATE;
Mat img_scene = SCENE_IMAGE;

  //-- Get the corners from the TEMPLATE_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - TEMPLATE_2 )
  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
}
  //-- Show detected matches
  imshow( "Good Matches & Object detection", img_matches );
////////////////////////////////////////////////////////////////////////////
 char key = cvWaitKey(10);
        if (key == 27) // ESC
            break;
//waitKey(0);
}

}



