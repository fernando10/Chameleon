// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <deque>
#include <iostream>
#include <vector>
#include <array>
#include <memory>
#include <sophus/se2.hpp>
#include <Eigen/Core>
#include "summersimulator/math_utils.h"

static Eigen::IOFormat kCleanFmt(4, 0, ", ", ";\n", "", "");
static Eigen::IOFormat kLongFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "");
static Eigen::IOFormat kLongCsvFmt(Eigen::FullPrecision, 0, ", ", "\n", "", "");

namespace summer
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
  double theta;  // [rad]
  double range;  // [m]
};

// TODO: Possibly remove this...may not be necessary
struct OdometryReading {
  double angle;  // [rad]
  double speed;  // [m/s]
};

struct OdometryMeasurement {
  double theta_1;
  double translation;
  double theta_2;

  OdometryMeasurement(): theta_1(0.), translation(0), theta_2(0.) {}

  OdometryMeasurement(double theta_1, double trans, double theta2):
    theta_1(theta_1), translation(trans), theta_2(theta2) {
  }
};

typedef std::vector<OdometryMeasurement> OdometryMeasurementVector;
typedef std::shared_ptr<std::vector<OdometryMeasurement>> OdometryMeasurementVectorPtr;
typedef Observation<OdometryReading> OdometryObservation;
typedef Observation<RangeFinderReading> RangeFinderObservation;
typedef std::deque<RangeFinderObservation> RangeFinderObservationDeque;

//------------------------POSES AND LANDMARKS------------------------//

typedef Eigen::Matrix3d PoseCovariance;

struct RobotPose {
  Sophus::SE2d pose;
  PoseCovariance covariance;

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
    pose.so2() = Sophus::SO2d(AngleWraparound(theta_new));
  }

  double x() const { return pose.translation().x(); }
  double& x() { return pose.translation().x(); }

  double y() const { return pose.translation().y(); }
  double& y() { return pose.translation().y(); }

  friend std::ostream& operator<<(std::ostream& os, const RobotPose& robot) {
    os << "theta: " << Rad2Deg(robot.theta()) << ", position: " << robot.translation().transpose();
    return os;
  }

};

typedef Eigen::Matrix2d LandmarkCovariance;

struct Landmark {
  static constexpr size_t kParamCount = 2;

  LandmarkCovariance covariance;
  uint64_t id;

  Landmark(): covariance(LandmarkCovariance::Identity()), id(0) {
  }

  double x() const { return data_[0]; }

  double y() const { return data_[1]; }

  double& x() { return data_[0]; }

  double& y() { return data_[1]; }

  void SetPosition(const double x, const double y) {
    data_[0] = x;
    data_[1] = y;
  }

  friend std::ostream& operator<<(std::ostream& os, const Landmark& lm) {
    os << "( " << lm.x() << ", " << lm.y() << " )";
    return os;
  }

  // Should only be used by ceres cost function
  double* data() { return data_.data(); }

 private:
  std::array<double, kParamCount> data_ = {{0.}};
};

typedef std::vector<Landmark> LandmarkVector;
typedef std::shared_ptr<LandmarkVector> LandmarkVectorPtr;
typedef std::vector<RobotPose> RobotPoseVector;
typedef std::shared_ptr<RobotPoseVector> RobotPoseVectorPtr;


//------------------------DATA STRUCTURES------------------------//

struct DebugData {
  RobotPoseVectorPtr ground_truth_poses;
  RobotPoseVectorPtr noisy_poses;
  RangeFinderObservationDeque noise_free_observations;
};

struct SimData {
  RangeFinderObservationDeque observations;
  DebugData debug;
};

} // namespace elninho
