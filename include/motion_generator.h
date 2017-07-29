// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include "types.h"
#include "util.h"
#include <unsupported/Eigen/Splines>

namespace elninho
{

class MotionGenerator {
 public:

  enum class MotionTypes {
    Rectangle,
    Circle,
    SineWave,
    StraightLine
  };

  struct MotionGeneratorOptions {
    double num_steps = 100;
    MotionTypes motion_type = MotionTypes::Rectangle;
  };

  MotionGenerator(const MotionGeneratorOptions& options);

  RobotPoseVectorPtr GenerateMotion() const;

 private:
  RobotPoseVectorPtr GenerateRectangularMotion() const;
  RobotPoseVectorPtr GenerateCircularMotion() const;
  RobotPoseVectorPtr GenerateSineWaveMotion() const;
  RobotPoseVectorPtr GenerateStraightLine() const;

  // 2D spline related
  typedef Eigen::Spline<double, 1> Spline1d;
  typedef Eigen::Spline2d Spline2d;
  typedef Eigen::Spline2d::PointType PointType;

  struct RobotSpline {
    Spline1d rotation_spline;
    Spline2d position_spline;
  };

  RobotSpline GenerateSplineFromWaypoints(const RobotPoseVectorPtr& waypoints) const;
  RobotPoseVectorPtr GeneratePosesFromSpline(const RobotSpline& spline) const;

  const MotionGeneratorOptions& options_;

  // rectangular motion parameters
  const double kRectangleWidth = 10; // [m]
  const double kRectangleLength = 20; // [m]
  const double kRectangleCornerPercent = 0.2; // [%] of length to start cornering at

  // circular motion parameters
  const double kCircleRadius = 10; // [m]

  // sine motion parameters
  const double kSineMagnitude = 5; // [m]


};

}
