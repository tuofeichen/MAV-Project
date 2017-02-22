
#ifndef DEBUG_H_
#define DEBUG_H_

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <fstream>
#include <string>

  // void initLog();
  // void logSlamNode(const Frame& , const Eigen::Isometry3d&,int, bool);
  // void logLpeNode(Matrix4f, double, int, bool);
  // bool readImage(Frame& frame, int id);
  // void saveImage(Frame frame, int id);
  // void showImage(Frame frame);
  // void logPoseGraphEnd(Frame frame, G2oPoseGraph graph, int nodeCnt);

  std::string exec(const char* cmd) {
      std::array<char, 128> buffer;
      std::string result;
      std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
      if (!pipe) throw std::runtime_error("popen() failed!");
      while (!feof(pipe.get())) {
          if (fgets(buffer.data(), 128, pipe.get()) != NULL)
              result += buffer.data();
      }
      return result;
  };

#endif
