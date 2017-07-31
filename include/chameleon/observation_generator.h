// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
// Generates observations of landmarks
//
#pragma once

#include "chameleon/types.h"
#include "chameleon/util.h"

namespace chameleon
{

class ObservationGenerator {
public:
  ObservationGenerator(const LandmarkVectorPtr map, const RobotPoseVectorPtr path);
  ObservationGenerator();

  // generates landmark observations for a given pose
  RangeFinderObservationVector GenerateObservations(size_t pose_idx) const;
  RangeFinderObservationVector GenerateNoisyObservations(size_t pose_index) const;


 void Reset(const LandmarkVectorPtr map, const RobotPoseVectorPtr path);
 void SetCovariance(Eigen::Matrix2d cov);

private:
  LandmarkVectorPtr map_;
  RobotPoseVectorPtr path_;
  Eigen::Matrix2d measurement_covariance_ = Eigen::Matrix2d::Identity();
};

} // namespace chameleon
