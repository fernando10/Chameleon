// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
// Generates 2D robot poses (theta, x, y) along a pre-determined path
// there is no noise added to the robot, these poses are meant to be used
// as the ground truth and to generate odometry
#pragma once

#include "chameleon/types.h"
#include "chameleon/util.h"
#include <unsupported/Eigen/Splines>

namespace chameleon
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
    RobotPose initial_position;  // all measurements are relative so we can set any initial position

  };

  PathGenerator(const PathGeneratorOptions& options);

  ///
  /// \brief GetRobot gets the robot pose at a specific timestamp
  /// if the timestamp is greater than the number of steps in the pre-generated
  /// path, it starts from the beginning again. As such ideally all simulated
  /// trajectories will end at the start position so it can be infinitely looped
  /// \param timestep step at which we want the robot pose
  /// \return ref of the robot pose
  ///
  RobotPose& GetRobot(size_t timestep);

  const RobotPoseVectorPtr GetRobotPath() const;

 private:

  ///
  /// \brief GeneratePath Generates full simulated path
  ///
  void GeneratePath();

  RobotPoseVectorPtr robot_poses_;

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

  // sine wave parameters
  const double kSineMagnitude = 10; // [m]
  const double kSineFrequency = 3;


};

}  // namespace chameleon
