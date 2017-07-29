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

  RobotPoseVector GenerateMotion();

 private:
  typedef Eigen::Spline<double, 1> Spline1d;
  typedef Eigen::Spline2d Spline2d;
  typedef Eigen::Spline2d::PointType PointType;

  struct RobotSpline {
    Spline1d rotation_spline;
    Spline2d position_spline;
  };

  RobotSpline GenerateSplineFromWaypoints(const RobotPoseVector& waypoints);
  RobotPoseVector GeneratePosesFromSpline(const RobotSpline& spline);
  RobotPoseVector GenerateRectangularMotion();
  RobotPoseVector GenerateStraightLine();

  const MotionGeneratorOptions& options_;

  const double kRectangleWidth = 20; // [m]
  const double kRectangleLength =40; // [m]


};

}
