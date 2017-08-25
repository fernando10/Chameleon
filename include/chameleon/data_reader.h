// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include "chameleon/types.h"
#include "chameleon/interpolation_buffer.h"

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

  // represents a keyframe and the measurements taken at that point in time
  struct KeyframeMeasurements {
    KeyframeMeasurements(double t): time(t){}
    double time;
    RangeFinderObservationVector meas;
    RobotPose ground_truth_pose;
    OdometryObservationVector odometry_meas;
  };

  struct UTIASData {
    UTIASData(){}
    LandmarkPtrMap gt_landmarks; // all the landmarks in the world frame
    LandmarkVectorPtr gt_landmarks_vec; // redundant storage for now due to backwards compatibility
    std::map<size_t, StatePtrVector> ground_truth_states; // ground truth robot poses for each robot
    std::map<size_t, std::vector<KeyframeMeasurements>> observations; // observations for each robot, binned by observation time
    std::map<size_t, OdometryObservationBufferPtr> odometry_buffers;  // odometry buffers for each robot

    std::map<size_t, std::vector<KeyframeMeasurements>> keyframes;
  };

  static void LoadG2o(const std::string& filename, G2oData* data);
  static void LoadUTIAS(const std::string&data_dir, UTIASData* data, double sample_time = 0.02, size_t num_robots = 5);


};

}  // namespace chameleon

