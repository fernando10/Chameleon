// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <deque>
#include <vector>
#include <sophus/se2.hpp>
#include <Eigen/Core>

static Eigen::IOFormat kCleanFmt(4, 0, ", ", ";\n", "", "");
static Eigen::IOFormat kLongFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "");
static Eigen::IOFormat kLongCsvFmt(Eigen::FullPrecision, 0, ", ", "\n", "", "");

namespace elninho
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

struct OdometryReading {
  double angle;  // [rad]
  double speed;  // [m/s]
};

typedef Observation<OdometryReading> OdometryObservation;
typedef Observation<RangeFinderReading> RangeFinderObservation;
typedef std::deque<RangeFinderObservation> RangeFinderObservationDeque;

//------------------------POSES AND LANDMARKS------------------------//

typedef Eigen::Matrix3d PoseCovariance;
struct RobotPose {
  Sophus::SE2d pose;
  PoseCovariance covariance;
  double time;

  RobotPose(): covariance(PoseCovariance::Identity()), time(0.) {
  }

};

typedef Eigen::Matrix2d LandmarkCovariance;

struct Landmark {
  double x;
  double y;
  LandmarkCovariance covariance;
  uint64_t id;

  Landmark(): x(0.), y(0.), covariance(LandmarkCovariance::Identity()), id(0) {
  }
};

typedef std::vector<RobotPose> RobotPoseVector;


} // namespace elninho
