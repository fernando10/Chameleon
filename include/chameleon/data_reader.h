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
    double range;
    double bearing;
    Eigen::Matrix2d information;
  };

  struct G2oData {
    StatePtrMap states;
    LandmarkPtrMap landmarks;
    std::vector<GraphEdgePoint> robot_landmark_edges;
    std::vector<GraphEdgeSE2> robot_robot_edges;
    size_t current_state = 0;
    size_t ts = 0;

    void Reset() {
      current_state = 0;
      ts = 0;
    }

    bool GetData(VictoriaParkData* data) {
      if (current_state >= states.size()) {
        return false;
      }

      data->timestamp = ts;

      // get all the observations from the current pose
      for (const GraphEdgePoint& edge : robot_landmark_edges) {
        if (edge.id1 == current_state) {
          RangeFinderObservation obs(0, RangeFinderReading(edge.bearing, edge.range));
          obs.observation.information = edge.information;
          data->observations.push_back(obs);
        }
      }
      // get the odometry from the curret pose
      for (const GraphEdgeSE2& edge : robot_robot_edges) {
        if (edge.id1 == current_state) {
          SE2OdometryMeasurement odometry;
          odometry.T = edge.measurement;
          odometry.information = edge.information;
          data->odometry = odometry;
          current_state = edge.id2;
          break;
        }
      }
      ts++;

      return true;

    }
  };

  struct UITASData {

  };

  static void LoadG2o(const std::string& filename, G2oData* data);
  static void LoadUITAS(const std::string& filename, G2oData* data );

private:


};

}  // namespace chameleon

