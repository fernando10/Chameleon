// Parts of this file were inspired by GTSAM's dataset.cpp file
// see https://bitbucket.org/gtborg/gtsam

#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include "glog/logging.h"
#include "fmt/format.h"
#include "fmt/ostream.h"
#include "chameleon/types.h"
#include "chameleon/data_reader.h"
#include "fmt/string.h"
#include "chameleon/id_provider.h"

#define LINESIZE 81920

namespace chameleon
{

static constexpr double kMeasurementTimeTolerance = 0.01;  // measurements within this tolearnce of each other will be grouped into the same keyframe

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
      edge.range = range;
      edge.bearing = bearing;
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

void DataReader::LoadUTIAS(const std::string &data_dir, UTIASData* data, double sample_time, size_t num_robots) {

  VLOG(1) << "Loading UTIAS dataset from dir: " << data_dir << " with sample time of: " << sample_time;


  std::vector<uint64_t> landmark_ids;
  std::map<uint64_t, uint64_t> barcode2subject;
  data->gt_landmarks_vec = std::make_shared<LandmarkVector>();

  ////////////////
  /// LOAD LANDMARKS
  ///
  std::string landmarks_file = fmt::format("{}/Landmark_Groundtruth.dat", data_dir);
  VLOG(1) << fmt::format("Loading landmarks from: {}", landmarks_file);
  std::ifstream input_stream(landmarks_file.c_str());

  if (!input_stream) {
    LOG(FATAL) << "Cannot load landmarks in file: " << landmarks_file;
  }

  while (!input_stream.eof()) {
    std::string first;
    int id = 0;
    if (!(input_stream >> first)) {
      break;
    }
    if (first == "#") {  // this line is a comment, ignore
      input_stream.ignore(LINESIZE, '\n');  // discard anything else on that line
      continue;
    } else {
      id = std::stoi(first);
    }

    double x = 0;
    double y = 0;
    double sigma_x = 0;
    double sigma_y = 0;
    input_stream >> x >> y >> sigma_x >> sigma_y;
    LandmarkPtr lm = std::make_shared<Landmark>(x, y);
    lm->id = id;
    landmark_ids.push_back((uint64_t)id);
    lm->covariance = Covariance2d(sigma_x, sigma_y, 0);
    data->gt_landmarks[id] = lm;
    data->gt_landmarks_vec->push_back(*lm);

    input_stream.ignore(LINESIZE, '\n');
    VLOG(2) << fmt::format("Landmark id: {}, pos: {}, {}, sigmas: {}, {}", id, x, y, sigma_x, sigma_y);
  }
  VLOG(1) << fmt::format("Added {} landmarks", data->gt_landmarks.size());
  // reserve these id's since they were provided externally and we dont want to create conflicting ids later on
  IdGenerator::Instance::ReserveIds(landmark_ids);

  ////////////////
  /// LOAD BARCODES
  ///
  std::string barcodes_file = fmt::format("{}/Barcodes.dat", data_dir);
  VLOG(1) << fmt::format("Getting barcodes from file {}", barcodes_file);
  input_stream.close();
  input_stream.clear();
  input_stream.open(barcodes_file.c_str());
  if (!input_stream) {
    LOG(FATAL) << "Cannot load barcodes from file: " << barcodes_file;
  }
  std::string first;
  int subject = 0;

  while (!input_stream.eof()) {
    if (!(input_stream >> first)) {
      break;
    }
    if (first == "#") {  // this line is a comment, ignore
      input_stream.ignore(LINESIZE, '\n');  // discard anything else on that line
      continue;
    } else {
      subject = std::stoi(first.c_str());
    }

    int barcode = 0;
    input_stream >> barcode;
    barcode2subject[(uint64_t)barcode] = (uint64_t)subject;
    input_stream.ignore(LINESIZE, '\n');
  }
  VLOG(1) << fmt::format("Read {} barcodes", barcode2subject.size());


  // now load the robot positions/odometry and measurements
  for (size_t robot_idx = 1; robot_idx <= num_robots; ++robot_idx) {
    VLOG(1) << "=================================";
    VLOG(1) << "Loading data for robot #" << robot_idx;

    std::map<double, size_t> timestep2keyframeIdx;

    ////////////////
    /// LOAD POSES
    ///
    std::string poses_file = fmt::format("{}/Robot{}_Groundtruth.dat", data_dir, robot_idx);
    VLOG(1) << fmt::format("Getting robot {} poses from file {}", robot_idx, poses_file);
    input_stream.close();
    input_stream.clear();
    input_stream.open(poses_file.c_str());
    if (!input_stream) {
      LOG(FATAL) << "Cannot load robot poses in file: " << poses_file;
    }
    std::string first;
    double ts = 0;

    while (!input_stream.eof()) {
      if (!(input_stream >> first)) {
        break;
      }
      if (first == "#") {  // this line is a comment, ignore
        input_stream.ignore(LINESIZE, '\n');  // discard anything else on that line
        continue;
      } else {
        ts = std::atof(first.c_str());
      }

      double x, y, theta;
      input_stream >> x >> y >> theta;

      KeyframeMeasurements::Ptr kf = std::make_shared<KeyframeMeasurements>();
      kf->time = ts;
      kf->ground_truth_pose.pose = Sophus::SE2d(theta, Eigen::Vector2d(x, y));
      data->keyframes[robot_idx].push_back(kf);
      timestep2keyframeIdx[ts] = data->keyframes[robot_idx].size()-1;

      input_stream.ignore(LINESIZE, '\n');
    }
    VLOG(1) << fmt::format("Added {} keyframes for robot idx {}", data->keyframes[robot_idx].size(), robot_idx);


    ////////////////
    /// LOAD MEASUREMENTS
    ///
    std::string measurement_file = fmt::format("{}/Robot{}_Measurement.dat", data_dir, robot_idx);
    VLOG(1) << fmt::format("Getting robot {} measurements from file {}", robot_idx, measurement_file);
    input_stream.close();
    input_stream.clear();
    input_stream.open(measurement_file);
    if (!input_stream) {
      LOG(FATAL) << "Cannot load robot measurements in file: " << measurement_file;
    }

    while (!input_stream.eof()) {
      if (!(input_stream >> first)) {
        break;
      }
      if (first == "#") {  // this line is a comment, ignore
        input_stream.ignore(LINESIZE, '\n');  // discard anything else on that line
        continue;
      } else {
        ts = std::atof(first.c_str());
      }

      int barcode = 0;
      double range = 0;
      double bearing = 0;
      input_stream >> barcode >> range >> bearing;

      if (barcode2subject.find(barcode) == barcode2subject.end()) {
        VLOG(1) << fmt::format("Barcode {} requested but not in barcodes map...should not happen.", barcode);
        continue;
      }

      // check if this is an observation of a landmark or of another robot
      if (data->gt_landmarks.find(barcode2subject.at(barcode)) == data->gt_landmarks.end()) {
        // this is an observation of another robot...discard for now
        continue;
      }

      RangeFinderObservation obs(ts, RangeFinderReading(barcode2subject.at(barcode), bearing, range));

      // check if we have a keyframe with this observations timestemp (we should)
      if (timestep2keyframeIdx.find(ts) != timestep2keyframeIdx.end()) {
        KeyframeMeasurements::Ptr& kf = data->keyframes.at(robot_idx)[timestep2keyframeIdx.at(ts)];
        kf->meas.push_back(obs);
      }
      input_stream.ignore(LINESIZE, '\n');
    }

    ////////////////
    /// LOAD ODOMETRY
    ///
    data->odometry_buffers[robot_idx] = std::make_shared<OdometryObservationBuffer>();
    std::string odometry_file = fmt::format("{}/Robot{}_Odometry.dat", data_dir, robot_idx);
    VLOG(1) << fmt::format("Getting robot {} odometry from file {}", robot_idx, odometry_file);
    input_stream.close();
    input_stream.clear();
    input_stream.open(odometry_file);
    if (!input_stream) {
      LOG(FATAL) << "Cannot load robot odometry in file: " << odometry_file;
    }

    while (!input_stream.eof()) {
      if (!(input_stream >> first)) {
        break;
      }
      if (first == "#") {  // this line is a comment, ignore
        input_stream.ignore(LINESIZE, '\n');  // discard anything else on that line
        continue;
      } else {
        ts = std::atof(first.c_str());
      }

      double forward_vel = 0;
      double omega = 0;
      input_stream >> forward_vel >> omega;

      OdometryObservation odo(ts, OdometryReading(forward_vel, omega));

      data->odometry_buffers[robot_idx]->AddElement(odo);
      if (timestep2keyframeIdx.find(ts) != timestep2keyframeIdx.end()) {
        KeyframeMeasurements::Ptr& kf = data->keyframes.at(robot_idx)[timestep2keyframeIdx.at(ts)];
        kf->odometry_meas.push_back(odo);
      }

      input_stream.ignore(LINESIZE, '\n');
    }
    VLOG(1) << fmt::format("Added {} odometry meas. for robot idx {}", data->odometry_buffers[robot_idx]->elements.size(), robot_idx);

  }

}

}  // namespace chameleon
