#ifndef INCLUDES_DEBUG_H_
#define INCLUDES_DEBUG_H_

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

  void initLog();
  void logSlamNode(const Frame& , const Eigen::Isometry3d&,int, bool);
  void logLpeNode(Matrix4f, double, int, bool);
  bool readImage(Frame& frame, int id);
  void saveImage(Frame frame, int id);
  void showImage(Frame frame);
  void logPoseGraphEnd(Frame frame, G2oPoseGraph graph, int nodeCnt);

#endif
