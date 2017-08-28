/**
 * @file ICP.cpp
 * @author Eric Dexheimer
 * @brief This file contains the implementation of the ICP class.
 *
 */
#include "DynamicObj.h"

namespace SLAM
{

DynamicObj::DynamicObj() : currCandidates(cv::Mat(480, 640,CV_8UC1, cvScalar(0))), prevCandidates(cv::Mat(480,640,CV_8UC1, cvScalar(0))),
                            outputDepth(cv::Mat(480,640,CV_16UC1, cvScalar(0)))
    {}

DynamicObj::~DynamicObj() {}

void DynamicObj::getObjCandidates(Eigen::Matrix4f tm, 
							Frame& currFrame, 
							Frame& prevFrame
							) 
{
    currCandidates = cv::Scalar(0);
    prevCandidates = cv::Scalar(0);

    /*cv::Mat map = currFrame.getDepth();
    double min, max;
    cv::minMaxIdx(map, &min, &max);
    cv::Mat adjMap;
    cv::convertScaleAbs(map, adjMap, 255 / max);
    cv::imshow("Out", adjMap);
    cv::waitKey(40);*/

    outputDepth = currFrame.getDepth();
    cv::Mat currDepth = currFrame.getDepth();
    cv::Mat prevDepth = prevFrame.getDepth();
    cv::Mat currGray = currFrame.getGray();
    cv::Mat prevGray = prevFrame.getGray();
    cv::GaussianBlur( currGray, currGray, cv::Size( 7, 7), 0, 0 );
    cv::GaussianBlur( prevGray, prevGray, cv::Size( 7, 7), 0, 0 );
    cv::Mat diff = currDepth - prevDepth;
    for (int i=0; i<currCandidates.rows; i++) {
        for (int j=0; j<currCandidates.cols; j++) {
            double currPZ = (static_cast<float> (currDepth.at<uint16_t>(i,j)) )*Frame::idepthScale;
            if (currPZ == 0)
                continue;
            double currPX = currPZ*(i-Frame::cx)/Frame::fx;
            double currPY = currPZ*(j-Frame::cy)/Frame::fy;
            Eigen::Vector4f currCoord;
            currCoord(2) = (static_cast<float> (currDepth.at<uint16_t>(i,j)) )*Frame::idepthScale; //z
            currCoord(0) = currCoord(2)*(i-Frame::cx)/Frame::fx; // x
            currCoord(1) = currCoord(2)*(j-Frame::cy)/Frame::fy; // y
            currCoord(3) = 1;  //homogenous coordinates
            Eigen::Vector4f prevCoord = tm*currCoord;

            int prevX = Frame::fx*prevCoord(0)/prevCoord(2) + Frame::cx;
            int prevY = Frame::fy*prevCoord(1)/prevCoord(2) + Frame::cy;
            //std::cout << prevX << " " << prevY << std::endl;
            if ( (prevX < 0) || (prevY < 0) || (prevX > currCandidates.rows) || (prevY > currCandidates.cols))
                continue;

            //std::cout << "i: " << i << " j: " << j << " x: " << prevX << " y: " << prevY << std::endl;

            //int currY = Frame::fx*currPoint.x/currPoint.z + Frame::cy;
            //int currX = Frame::fy*currPoint.y/currPoint.z + Frame::cx;

            //std::cout << currGray.at<uint16_t>(i,j)-prevGray.at<uint16_t>(i,j) << std::endl;
            if (abs(currGray.at<uint16_t>(i,j)-prevGray.at<uint16_t>(prevX,prevY)) > intensityThreshold) {
                double diff = (currDepth.at<uint16_t>(i,j) - prevDepth.at<uint16_t>(prevX,prevY))*Frame::idepthScale;
                if (!((diff < 10) && (diff > -10)) || currDepth.at<uint16_t>(i,j)==0) // || prevDepth.at<uint16_t>(prevX,prevY)==0)  // get rid of NaN values, comparison to NaN was not qorking
                    continue;                    // using 10 arbitrarily since max depth of ASUS is around 5 m
                //std::cout << diff << std::endl;
                double percentDiff = diff/( (double) currDepth.at<uint16_t>(i,j)*Frame::idepthScale);
                //std::cout << percentDiff << std::endl;
                //std::cout << i << " " << j << std::endl;
                if (diff > dynamicDepthThreshold) {
                    //prevCandidates.at<uint16_t>(x,y) = 255;
                    currCandidates.at<uint8_t>(i,j) = 0;
                    //std::cout << "x: " << x << " y: " << y << " i: " << i << " j: " << j << std::endl;
                }
                else if (diff <  -dynamicDepthThreshold) {
                    //prevCandidates.at<uint16_t>(x,y) = 0;
                    //std::cout << currDepth.at<uint16_t>(i,j) << " " << prevDepth.at<uint16_t>(i,j) << std::endl;
                    currCandidates.at<uint8_t>(i,j) = 255;
                }
                else {
                    //prevCandidates.at<uint16_t>(x,y) = 0;
                    currCandidates.at<uint8_t>(i,j) = 0;
                }
            }
        }
    }
    //cv::erode(currCandidates, currCandidates, cv::Mat());
    //cv::dilate(currCandidates, currCandidates, cv::Mat());

    //Eigen::Matrix4f tm = transformationMatrix;
	//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
  	// You can either apply transform_1 or transform_2; they are the same
    //pcl::transformPointCloud(*currCloud, *transformedCloud, tm);

    //std::cout << transformedCloud->width << " " << transformedCloud->height << std::endl;
    //std::cout << currCandidates.cols << " " << currCandidates.rows << std::endl;
	//cv::Mat diffDepth(currDepth.rows, currDepth.cols, CV_64F, 0);
	/*for (size_t i=0; i<(currCloud->height); i++) {
		for (size_t j=0; j<(currCloud->width); j++) {
			pcl::PointXYZRGBNormal currPoint;
            currPoint = currCloud->points[i*currCloud->width + j];
            //std::cout << i*transformedCloud->width + j << std::endl;
            //currCandidates.at<uint16_t>(0,0) = 0;
            int currY = Frame::fx/2*currPoint.x/currPoint.z + 59.5;
            int currX = Frame::fy/2*currPoint.y/currPoint.z + 79.5;
            //std::cout << currX << " " << currY << std::endl;
            if (((currPoint.z < 10) && (currPoint.z > -10)) 
                && (currX >= 0) && (currX < currCandidates.rows) 
                && (currY >= 0) && (currY < currCandidates.cols)) {
                currCandidates.at<uint16_t>(currX,currY) = currPoint.z*255/5;
            }
            //else
            //    currCandidates.at<uint16_t>(159-j,119-i) = 0.0;
            //std::cout << i << " " << j << std::endl;

			/*int y = ((currPoint.y*Frame::fx/2)/currPoint.z + 59.5);
			int x = ((currPoint.x*Frame::fy/2)/currPoint.z + 79.5);
            if (x<0 || x>=transformedCloud->width || y<0 || y>=transformedCloud->height)
                continue;
			pcl::PointXYZRGBNormal prevPoint = prevCloud->points[x*prevCloud->width + y];
			double diff = currPoint.z - prevPoint.z;
            double percentDiff = diff/prevPoint.z;
			if (!((diff < 10) && (diff > -10)))	 // get rid of NaN values, comparison to NaN was not qorking
				continue;	*/                     // using 10 arbitrarily since max depth of ASUS is around 5 m
            //std::cout << currPoint.z << " " << prevPoint.z << " " << diff << " " << std::endl;
			/*if (percentDiff > dynamicDepthThreshold) {
				prevCandidates.at<uint16_t>(x,y) = 255;
				currCandidates.at<uint16_t>(i,j) = 0;
                std::cout << "x: " << x << " y: " << y << " i: " << i << " j: " << j << std::endl;
			}
			else if (percentDiff <  -dynamicDepthThreshold) {
				prevCandidates.at<uint16_t>(x,y) = 0;
				currCandidates.at<uint16_t>(i,j) = 255;
			}
			else {
				prevCandidates.at<uint16_t>(x,y) = 0;
				currCandidates.at<uint16_t>(i,j) = 0;
			}
		}
	}*/

}

void DynamicObj::clusterPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud) {

    pcl::copyPointCloud(*cloud, *outputCloud);

	// Segment Planes
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::Label> mps;
    std::vector<pcl::PlanarRegion<pcl::PointXYZRGBNormal>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBNormal> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices, boundary_indices, label_indices;  
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    mps.setMinInliers(2000);
    mps.setAngularThreshold (pcl::deg2rad (5.0)); //3 degrees
    mps.setDistanceThreshold (0.05); //2cm
    //mps.setMaximumCurvature(0.01);
    mps.setInputNormals (cloud);
    mps.setInputCloud (cloud);
    mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

    std::vector<bool> plane_labels;
    plane_labels.resize (label_indices.size (), false);
    //std::cout << "number of label_indices: " << label_indices.size() << std::endl;
    for (size_t i = 0; i < label_indices.size (); i++)
    {
      if (label_indices[i].indices.size () > 2000)
      {
        //std::cout << label_indices[i].indices.size() << std::endl;
        //std::cout << "found plane" << std::endl;
        plane_labels[i] = true;
      }
    }
    
    //std::cout << "plane labels size: " << plane_labels.size() << std::endl;
    //std::cout << "labels size: " << labels->size() << std::endl;

    pcl::EuclideanClusterComparator<pcl::PointXYZRGBNormal, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator;
    euclidean_cluster_comparator = pcl::EuclideanClusterComparator<pcl::PointXYZRGBNormal, pcl::Normal, pcl::Label>::Ptr (new pcl::EuclideanClusterComparator<pcl::PointXYZRGBNormal, pcl::Normal, pcl::Label> ());
    euclidean_cluster_comparator->setInputCloud (cloud);
    euclidean_cluster_comparator->setLabels (labels);
    euclidean_cluster_comparator->setExcludeLabels (plane_labels);
    euclidean_cluster_comparator->setDistanceThreshold(0.1f, false);

    pcl::PointCloud<pcl::Label> euclidean_labels;
    std::vector<pcl::PointIndices> euclidean_label_indices;
    pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGBNormal,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator);
    euclidean_segmentation.setInputCloud (cloud);
    euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
    
    //cv::Mat cluster_image = cv::Mat(cloud->height, cloud->width, CV_8UC1, cvScalar(0));
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> clusters;
    //std::cout << euclidean_label_indices.size() << std::endl;
    /*for (size_t i = 0; i < euclidean_label_indices.size (); i++)
    {
      if (euclidean_label_indices[i].indices.size () < 25)      // check here
        continue;
      /*if (euclidean_label_indices[i].indices.size () > 100)
      {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::copyPointCloud(*cloud,euclidean_label_indices[i].indices,*cluster);
        clusters.push_back(cluster);
      } //
      for (size_t j=0; j<euclidean_label_indices[i].indices.size(); j++) {
      	cluster_image.at<uint8_t>(euclidean_label_indices[i].indices[j]/cloud->width,euclidean_label_indices[i].indices[j]%cloud->width) = 255;
      }
    }*/

    //cv::Mat se = cv::Mat(9,9,CV_8UC1,cvScalar(1));
    //cv::erode(cluster_image, cluster_image, se); // done to prevent dynamic objects from connecting with static
    //cv::dilate(cluster_image,cluster_image,se);
    /*cv::imshow( "Display window2", cluster_image );  

    cv::Mat intersection = cv::Mat(cluster_image.rows, cluster_image.cols, CV_8UC1, cvScalar(0));
    cv::bitwise_and(cluster_image,currCandidates,intersection);

    cv::Mat cc_labels, stats, centroids;
    //cv::erode(intersection, intersection, cv::Mat());
    //cv::dilate(intersection, intersection, cv::Mat());
    //cv::erode(intersection, intersection, cv::Mat());
    //cv::dilate(intersection, intersection, cv::Mat());
    int nlabels = cv::connectedComponentsWithStats(intersection, cc_labels, stats, centroids);

    cv::imshow("Intersection", intersection);
    cv::waitKey(10);*/

    /*cv::Mat maskPointCloud = cv::Mat(cluster_image.rows, cluster_image.cols, CV_8UC1, cvScalar(0));
    for (int i=1; i<nlabels; i++) {
        bool found_index = false;
		if (stats.at<int>(i,cv::CC_STAT_AREA) > 25) {
            //std::cout << nlabels << std::endl;
			//compare(cc_labels, i, maskPointCloud, cv::CMP_EQ);	
            //std::cout << maskPointCloud.cols << " " << maskPointCloud.rows << std::endl;
            //cv::imshow("maskPointCloud", maskPointCloud);
            //cv::waitKey(0);	// accumulate clusters, fix this logic
            int count = 0;
            for (int j=0; j<cc_labels.rows; j++) {
                if (found_index)
                    break;
                for (int k=0; k<cc_labels.cols; k++) {
                    if (cc_labels.at<uint16_t>(j, k)==i) {
                        found_index = true;   
                        uint32_t pc_label = euclidean_labels.points[j*cloud->width+k].label;
                        std::cout << euclidean_label_indices.size() << " " << pc_label << std::endl;
                        if (pc_label == 4294967295)
                            break;
                        std::vector<uint32_t> overlap_indices;
                        std::vector<uint32_t> dynamic_indices;
                        for (size_t row=0; row<euclidean_labels.height; row++) {
                            for(size_t col=0; col<euclidean_labels.width; col++) {
                                if (pc_label == euclidean_labels[row*euclidean_labels.width+col].label)
                                {
                                    //outputCloud->points[j*cloud->width + k].x = std::numeric_limits<float>::quiet_NaN();   
                                    //outputCloud->points[j*cloud->width + k].y = std::numeric_limits<float>::quiet_NaN();  
                                    //outputCloud->points[j*cloud->width + k].z = std::numeric_limits<float>::quiet_NaN(); 
                                    dynamic_indices.push_back(row*cloud->width + col);
                                    if (cc_labels.at<uint16_t>(row,col) == i)
                                        overlap_indices.push_back(row*cloud->width + col);
                                }
                            } 
                        }
                        if (dynamic_indices.size() > 100) {
                            for (size_t l=0; l<dynamic_indices.size(); l++) {
                                outputCloud->points[dynamic_indices[l]].r = 255;
                                outputCloud->points[dynamic_indices[l]].g = 0;
                                outputCloud->points[dynamic_indices[l]].b = 0;
                            }
                        }
                        std::cout << "points removed: " << count << std::endl;
                    }
                    if (found_index)
                        break;
                }
            }
            //cv::imshow("Display window4", maskPointCloud);

		}
	}*/

    for (size_t i=0; i<euclidean_label_indices.size(); i++) {
        if (euclidean_label_indices[i].indices.size() > 25) {
            cv::Mat cluster_image = cv::Mat(cloud->height, cloud->width, CV_8UC1, cvScalar(0));
            for (size_t j=0; j<euclidean_label_indices[i].indices.size(); j++) {
                cluster_image.at<uint8_t>(euclidean_label_indices[i].indices[j]/cloud->width,euclidean_label_indices[i].indices[j]%cloud->width) = 255;
            } // watch precision of this, may need to convert using focal length
            //cv::dilate(currCandidates, currCandidates, cv::Mat());
            cv::Mat intersection = cv::Mat(cluster_image.rows, cluster_image.cols, CV_8UC1, cvScalar(0));
            cv::bitwise_and(cluster_image,currCandidates,intersection);
            if (cv::countNonZero(intersection) > 25) {
                for (size_t j=0; j<euclidean_label_indices[i].indices.size(); j++) {
                    outputCloud->points[euclidean_label_indices[i].indices[j]].r = 255;
                    outputCloud->points[euclidean_label_indices[i].indices[j]].g = 0;
                    outputCloud->points[euclidean_label_indices[i].indices[j]].b = 0;
                    outputDepth.at<uint16_t>(euclidean_label_indices[i].indices[j]/cloud->width,euclidean_label_indices[i].indices[j]%cloud->width) = 0;
                }
            }

        }
    }

    /*double min, max;
    cv::minMaxIdx(outputDepth, &min, &max);
    cv::Mat adjMap;
    cv::convertScaleAbs(outputDepth, adjMap, 255 / max);
    cv::imshow("Output Depth", adjMap);
    cv::waitKey(10);*/


}

void DynamicObj::dynamicObjectRemoval(Eigen::Matrix4f tm, 
                            Frame currFrame, 
                            Frame prevFrame) {

    this->getObjCandidates(tm, currFrame, prevFrame);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    SLAM::FrameToPcConverter::getColorPC(currFrame,*colorCloud);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr currCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud(*colorCloud, *currCloud);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(currCloud);
    ne.compute(*currCloud);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    this->clusterPointCloud(currCloud,outputCloud);

}

}