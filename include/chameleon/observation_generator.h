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

  // generates landmark observations for a given pose
  RangeFinderObservationVector GenerateObservations(size_t pose_idx);

 void Reset(const LandmarkVectorPtr map, const RobotPoseVectorPtr path);



private:
  LandmarkVectorPtr map_;
  RobotPoseVectorPtr path_;
};

} // namespace chameleon
