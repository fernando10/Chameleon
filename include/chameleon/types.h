// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <deque>
#include <iostream>
#include <vector>
#include <array>
#include <map>
#include <memory>
#include <sophus/se2.hpp>
#include <cstdint>
#include <Eigen/Core>
#include "chameleon/util.h"
#include "chameleon/math_utils.h"

static Eigen::IOFormat kCleanFmt(4, 0, ", ", ";\n", "", "");
static Eigen::IOFormat kLongFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "");
static Eigen::IOFormat kLongCsvFmt(Eigen::FullPrecision, 0, ", ", "\n", "", "");

namespace chameleon
{

//------------------------OBSERVATION TYPES------------------------//
template<class T>
struct Observation {
  double timestamp;
  T observation;

  Observation() : timestamp(0.0) {
  }

  Observation(const double& time, const T& obs) :
    timestamp(time), observation(obs) {
  }
};

struct RangeFinderReading {
  static constexpr size_t kMeasurementDim = 2;
  static constexpr size_t kIndexRange = 0;
  static constexpr size_t kIndexBearing = 1;
  RangeFinderReading(double theta, double range): theta(theta), range(range) {
  }

  RangeFinderReading(uint64_t lm_id_, double theta, double range): RangeFinderReading(theta, range) {
    lm_id = lm_id_;
  }


  RangeFinderReading operator+(const Eigen::Vector2d& rhs) const {
    return RangeFinderReading(lm_id, theta + rhs[0], range + rhs[1]);
  }
  double theta;  // [rad]
  double range;  // [m]
  uint64_t lm_id = 0; // landmark id (only populated when generating simulated data
};

struct OdometryMeasurement {
  static constexpr size_t kMeasurementDim = 3;
  double theta_1;
  double translation;
  double theta_2;

  OdometryMeasurement(): theta_1(0.), translation(0), theta_2(0.) {}

  OdometryMeasurement(double theta_1, double trans, double theta2):
    theta_1(theta_1), translation(trans), theta_2(theta2) {
  }
};

typedef Eigen::Matrix<double, RangeFinderReading::kMeasurementDim,
RangeFinderReading::kMeasurementDim> RangeFinderCovariance;

typedef Eigen::Matrix<double, OdometryMeasurement::kMeasurementDim,
OdometryMeasurement::kMeasurementDim> OdometryCovariance;

typedef std::vector<OdometryMeasurement> OdometryMeasurementVector;
typedef std::shared_ptr<std::vector<OdometryMeasurement>> OdometryMeasurementVectorPtr;
typedef Observation<RangeFinderReading> RangeFinderObservation;
typedef std::vector<RangeFinderObservation> RangeFinderObservationVector;
typedef std::map<size_t, RangeFinderObservationVector> RangeFinderObservationVectorMap;

//------------------------POSES AND LANDMARKS------------------------//

typedef Eigen::Matrix2d LandmarkCovariance;

struct Landmark {
  static constexpr size_t kLandmarkDim = 2;
  LandmarkCovariance covariance;
  uint64_t id;

  Landmark(): covariance(LandmarkCovariance::Identity()), id(0) {
  }
  Landmark(uint64_t id_, double x, double y): Landmark(x, y) {
    id = id_;
  }
  Landmark(double x, double y): Landmark() {
    data_[0] = x;
    data_[1] = y;
  }

  Landmark(const Landmark& lm) {
    id = lm.id;
    data_[0] = lm.x();
    data_[1] = lm.y();
  }

  Landmark(Eigen::Vector2d vec): Landmark() {
    data_[0] = vec[0];
    data_[1] = vec[1];
  }

  double x() const { return data_[0]; }

  double y() const { return data_[1]; }

  double& x() { return data_[0]; }

  double& y() { return data_[1]; }

  void SetPosition(const double x, const double y) {
    data_[0] = x;
    data_[1] = y;
  }

  Eigen::Vector2d vec() const {
    return data_;
  }

  friend std::ostream& operator<<(std::ostream& os, const Landmark& lm) {
    os << "( " << lm.x() << ", " << lm.y() << " )";
    return os;
  }

  // Should only be used by ceres cost function
  double* data() { return data_.data(); }

private:
  Eigen::Vector2d data_ = Eigen::Vector2d::Zero();
  //std::array<double, kParamCount> data_ = {{0.}};
};


typedef Eigen::Matrix3d PoseCovariance;
struct RobotPose {
  Sophus::SE2d pose;
  PoseCovariance covariance;
  double field_of_view = Deg2Rad(90);  // [rad]
  double range = 10;  // [m]

  // Default constructor
  RobotPose(): covariance(PoseCovariance::Identity()) {
  }

  // Construct from angle and position
  RobotPose(const double theta, const Eigen::Vector2d position) : RobotPose() {
    pose = Sophus::SE2d(theta, position);
  }

  // Construct from an SE2d transform
  RobotPose(const Sophus::SE2d p): RobotPose() {
    pose = p;
  }

  // Construct from an SO2d and position
  RobotPose(const Sophus::SO2d rot, const Eigen::Vector2d pos): RobotPose() {
    pose = Sophus::SE2d(rot, pos);
  }

  // Construct from an angle and x, y
  RobotPose(const double theta, const double x, const double y): RobotPose() {
    pose = Sophus::SE2d(theta, Eigen::Vector2d(x, y));
  }

  // Construct from another RobotPose
  RobotPose(const RobotPose& other) {
    pose = other.pose;
    covariance = other.covariance;
  }

  // Multiplication with another robot pose
  RobotPose operator*(const RobotPose& rhs) const {
    return RobotPose(pose * rhs.pose);
    // TODO: propagate covariance
  }

  // Multiplication with an SE2d transform
  RobotPose operator*(const Sophus::SE2d& rhs) const {
    return RobotPose(pose * rhs);
  }

  // Multiplication with an landmark
  Landmark operator*(const Landmark& rhs) const {
    return Landmark(util::DeHomogenizeLandmark<double>(pose.matrix() * util::HomogenizeLandmark<double>(rhs.vec())));
  }

  // Assignment -- copy
  RobotPose& operator=(const RobotPose& rhs) {
    pose = rhs.pose;
    covariance = rhs.covariance;
    return *this;
  }

  const Eigen::Vector2d& translation() const {
    return pose.translation();
  }

  const Sophus::SO2d& SO2() const {
    return pose.so2();
  }

  double theta() const { return pose.so2().log(); }

  // we don't store theta directly so can't easily return a reference like we do for x and y
  void SetTheta(const double theta_new) {
    pose.so2() = Sophus::SO2d(AngleWraparound<double>(theta_new));
  }

  double x() const { return pose.translation().x(); }
  double& x() { return pose.translation().x(); }

  double y() const { return pose.translation().y(); }
  double& y() { return pose.translation().y(); }

  // only to be used by ceres parameter block
  double* data() { return pose.data(); }

  friend std::ostream& operator<<(std::ostream& os, const RobotPose& robot) {
    os << "theta: " << Rad2Deg(robot.theta()) << ", position: " << robot.translation().transpose();
    return os;
  }

};

typedef std::shared_ptr<Landmark> LandmarkPtr;
typedef std::map<uint64_t, LandmarkPtr> LandmarkPtrMap;
typedef std::vector<Landmark> LandmarkVector;
typedef std::shared_ptr<LandmarkVector> LandmarkVectorPtr;
typedef std::vector<RobotPose> RobotPoseVector;
typedef std::shared_ptr<RobotPoseVector> RobotPoseVectorPtr;
typedef std::map<size_t, uint64_t> DataAssociationMap;


//------------------------DATA STRUCTURES------------------------//

//struct DebugData {
//  DebugData() {
//    ground_truth_map = std::make_shared<LandmarkVector>();
//    ground_truth_poses = std::make_shared<RobotPoseVector>();
//    noisy_poses = std::make_shared<RobotPoseVector>();
//    noise_free_odometry = std::make_shared<OdometryMeasurementVector>();
//    noisy_odometry = std::make_shared<OdometryMeasurementVector>();
//  }

//  RobotPoseVectorPtr ground_truth_poses;
//  RobotPoseVectorPtr noisy_poses;
//  OdometryMeasurementVectorPtr noise_free_odometry;
//  OdometryMeasurementVectorPtr noisy_odometry;
//  LandmarkVectorPtr ground_truth_map;
//  RangeFinderObservationVectorMap noise_free_observations;
//  RangeFinderObservationVectorMap noisy_observations;
//};

// debug data for a single timestep
struct DebugData {
  DebugData() {
    ground_truth_map = std::make_shared<LandmarkVector>();
  }

  RobotPose ground_truth_pose;
  RobotPose noisy_pose;
  OdometryMeasurement noise_free_odometry;
  OdometryMeasurement noisy_odometry;
  LandmarkVectorPtr ground_truth_map;
  RangeFinderObservationVector noise_free_observations;
  RangeFinderObservationVector noisy_observations;
};

struct SimData {
  std::vector<size_t> times;
  DebugData debug;
};

struct VictoriaParkData {
  // TODO
};

struct RobotData {
  double timestamp;
  DebugData debug;
  // this is all that is available to the estimation algorithm (below)
  OdometryMeasurement odometry;
  RangeFinderObservationVector observations;
};

//------------------------STATE ESTIMATION STUFF------------------------//
struct State {
  State():id(0), timestamp(0) {}
  uint64_t id;
  RobotPose robot;
  double timestamp;
  double* data() { return robot.data(); }
  int DoF() { return robot.pose.DoF; }
};

typedef std::shared_ptr<State> StatePtr;
typedef std::map<uint64_t, StatePtr> StatePtrMap;
typedef std::vector<StatePtr> StatePtrVector;
typedef std::multimap<uint64_t, uint64_t> State2Landmark_Multimap;
typedef std::multimap<uint64_t, uint64_t> Landmark2State_MultiMap;

struct EstimatedData {
  //const StatePtr state;
  StatePtrMap states;
  LandmarkPtrMap landmarks;
};

} // namespace chameleon
