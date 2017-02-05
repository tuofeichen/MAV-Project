
#ifndef DEBUG_H_
#define DEBUG_H_

#include "Frame.h"
#include "G2oPoseGraph.h"
#include "Mapping.h"
#include "Eigen/Core"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace SLAM;

  void logPoseGraphNode(const Frame& , const Eigen::Isometry3d&,int, bool);
  bool readImage(Frame& frame, int id);
  void saveImage(Frame frame, int id);
  void showImage(Frame frame);
  // void logPoseGraphEnd(Frame frame, G2oPoseGraph graph, int nodeCnt);

#endif
