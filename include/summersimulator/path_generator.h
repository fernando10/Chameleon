// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
// Generates 2D robot poses (theta, x, y) along a pre-determined path
// there is no noise added to the robot, these poses are meant to be used
// as the ground truth and to generate odometry
#pragma once

#include "summersimulator/types.h"
#include "summersimulator/util.h"
#include <unsupported/Eigen/Splines>

namespace summer
{

class PathGenerator {
 public:

  enum class PathTypes {
    Rectangle,
    Circle,
    SineWave,
    StraightLine
  };

  struct PathGeneratorOptions {
    size_t num_steps = 100;
    PathTypes motion_type = PathTypes::Rectangle;
  };

  PathGenerator(const PathGeneratorOptions& options);

  RobotPoseVectorPtr GeneratePath() const;

 private:
  RobotPoseVectorPtr GenerateRectangularPath() const;
  RobotPoseVectorPtr GenerateCircularPath() const;
  RobotPoseVectorPtr GenerateSineWavePath() const;
  RobotPoseVectorPtr GenerateStraightLinePath() const;

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

  const PathGeneratorOptions& options_;

  // straight line parameters
  const double kLineLength = 20.; // [m]

  // rectangular motion parameters
  const double kRectangleWidth = 10.; // [m]
  const double kRectangleLength = 20.; // [m]
  const double kRectangleCornerPercent = 0.2; // [%] of length to start cornering at

  // circular motion parameters
  const double kCircleRadius = 20.; // [m]


};

}
