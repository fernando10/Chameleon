// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
// Generates motions, or odometry given a ground truth path
// This class is tasked with:
//    - Generating odometry given poses at times t-1 and t,
//    - Integrating odometry (with or without noise) and propagating pose t-1 to pose t
//
#pragma once

#include "summersimulator/types.h"
#include "summersimulator/util.h"

namespace summer{

class MotionGenerator {
public:

  MotionGenerator(const RobotPoseVectorPtr robot_poses);
  MotionGenerator() {}

  void SetPath(const RobotPoseVectorPtr robot_poses);
  RobotPoseVectorPtr GetPath();

  OdometryMeasurement GenerateOdometryMeasurement(size_t step);

  RobotPose PropagateMeasurement(const OdometryMeasurement& meas, const RobotPose& current_pose);

private:
  RobotPoseVectorPtr robot_poses_;
};

}  // namespace elninho
