// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
// Generates odometry given a ground truth path
// This class is tasked with:
//    - Generating odometry given poses at times t-1 and t,
//    - Integrating odometry (with or without noise) and propagating pose t-1 to pose t
//
#pragma once

#include "chameleon/types.h"
#include "chameleon/util.h"

namespace chameleon{

class OdometryGenerator {
public:

  OdometryGenerator(const RobotPoseVectorPtr robot_poses);
  OdometryGenerator(const Eigen::Vector4d odometry_noise);
  OdometryGenerator() {}

  void SetPath(const RobotPoseVectorPtr robot_poses);

  OdometryMeasurement GenerateNoiseFreeOdometryMeasurement(size_t step) const;
  OdometryMeasurement GenerateNoisyOdometryMeasurement(size_t step) const;

  RobotPose PropagateMeasurement(const OdometryMeasurement& meas, const RobotPose& current_pose) const;
  RobotPose& PropagateMeasurement(const OdometryMeasurement& meas);


private:
  OdometryMeasurementVectorPtr GenereteOdometry(bool noisy = false) const;

  RobotPoseVectorPtr robot_poses_;
  RobotPoseVectorPtr noisy_robot_poses_;
  Eigen::Vector4d odometry_noise_ = Eigen::Vector4d::Zero();
};

}  // namespace chameleon
