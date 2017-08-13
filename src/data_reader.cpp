// Parts of this file were inspired by GTSAM's dataset.cpp file
// see https://bitbucket.org/gtborg/gtsam

#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include "glog/logging.h"
#include "fmt/format.h"
#include "fmt/ostream.h"

#include "chameleon/data_reader.h"

#define LINESIZE 81920

namespace chameleon
{

void DataReader::LoadG2o(const std::string &filename, G2oData* data) {

  std::ifstream input_stream(filename.c_str());
  if (!input_stream) {
    LOG(FATAL) << "Cannot load filename" << filename;
  }

  std::string tag;

  while (!input_stream.eof()) {
    if (!(input_stream >> tag)) {
      break;
    }

    // parse the robot pose vertices
    if (tag == "VERTEX_SE2") {
      size_t id;
      double x, y, theta;
      input_stream >> id >> x >> y >> theta;

      // create a new state
      StatePtr state = std::make_shared<State>();
      state->robot = RobotPose(theta, x, y);
      state->id = id;

      // and add it to the map
      data->states[id] = state;
    }

    // parse the landmark vertices
    if (tag == "VERTEX_XY") {
      size_t id;
      double x, y;
      input_stream >> id >> x >> y;

      // create a new landmark
      LandmarkPtr lm = std::make_shared<Landmark>();
      lm->SetPosition(x, y);
      lm->id = id;

      // add it to the map
      data->landmarks[id] = lm;
    }
    input_stream.ignore(LINESIZE, '\n');  // discard anything else on that line
  }
  input_stream.clear();  // clear any flags that may have been set (EoF, etc)
  input_stream.seekg(0, input_stream.beg);


  // now parse the factors
  double bearing, range, bearing_std, range_std;
  size_t id1, id2;
  while (!input_stream.eof()) {
    if (!(input_stream >> tag)) {
      break;
    }

    if (tag == "EDGE_SE2") {

      double x, y, theta;
      input_stream >> id1 >> id2 >> x >> y >> theta;
      Sophus::SE2d meas(theta, Eigen::Vector2d(x, y));

      // now read in the information matrix
      double v1, v2, v3, v4, v5, v6;
      input_stream >> v1 >> v2 >> v3 >> v4 >> v5 >> v6;
      Eigen::Matrix3d inf;
      inf << v1, v2, v3, v2, v4, v5, v3, v5, v6;

      GraphEdgeSE2 edge;
      edge.id1 = id1;
      edge.id2 = id2;
      edge.information = inf;
      edge.measurement = meas;

      data->robot_robot_edges.push_back(edge);
    }

    if (tag == "EDGE_SE2_XY") {
      double x, y;
      double v1, v2, v3;
      input_stream >> id1 >> id2 >> x >> y >> v1 >> v2 >> v3;

      // convert x, y to bearing, range
      bearing = std::atan2(y, x);
      range = std::sqrt(Square(x) + Square(y));

      // TODO: check if this covariance conversion makes sense for the victoria park data
      if (std::abs(v1 - v3) < 1e-4) {
        bearing_std = std::sqrt(v1 / 10.0);
        range_std = std::sqrt(v1);
      } else {
        bearing_std = 1;
        range_std = 1;
      }

      Eigen::Matrix2d inf = Eigen::Matrix2d::Identity();
      inf(RangeFinderReading::kIndexBearing, RangeFinderReading::kIndexBearing) = bearing_std;
      inf(RangeFinderReading::kIndexRange, RangeFinderReading::kIndexRange) = range_std;


      GraphEdgePoint edge;
      edge.id1 = id1;
      edge.id2 = id2;
      edge.measurement = Eigen::Vector2d(x, y);
      edge.information = inf;

      data->robot_landmark_edges.push_back(edge);
    }
  }

  LOG(INFO) << fmt::format("Loaded {} poses, {} landmarks, {} odometry constraints and {} landmark observations from file: {}",
                           data->states.size(),
                           data->landmarks.size(),
                           data->robot_robot_edges.size(),
                           data->robot_landmark_edges.size(),
                           filename);

}

}  // namespace chameleon
