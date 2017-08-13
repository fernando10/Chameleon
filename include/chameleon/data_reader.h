// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include "chameleon/types.h"

namespace chameleon
{

class DataReader {
public:

  struct GraphEdgeSE2 {
    uint64_t id1;
    uint64_t id2;
    Sophus::SE2d measurement;
    Eigen::Matrix3d information;
  };

  struct GraphEdgePoint {
    uint64_t id1;
    uint64_t id2;
    Eigen::Vector2d measurement;
    Eigen::Matrix2d information;
  };

  struct G2oData {
    StatePtrMap states;
    LandmarkPtrMap landmarks;
    std::vector<GraphEdgePoint> robot_landmark_edges;
    std::vector<GraphEdgeSE2> robot_robot_edges;
  };

  static void LoadG2o(const std::string& filename, G2oData* data);

private:


};

}  // namespace chameleon

