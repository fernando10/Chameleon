// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include "chameleon/types.h"
#include "chameleon/util.h"

namespace chameleon
{

enum class WorldTypes {
  RectangularField,
  StraightLine,
  TwoParallelLines,
  RandomTrees
};

class WorldGenerator {
 public:
  ///
  /// \brief WorldGenerator
  /// Default Constructor
  WorldGenerator(){}

  LandmarkVectorPtr GenerateWorld(const RobotPoseVectorPtr& robot_poses);
  LandmarkVectorPtr GetWorld() const;

private:

  LandmarkVectorPtr map_;
  double GetTotalDistanceTraveled(const RobotPoseVectorPtr& poses);
  const double kLandmarkDensity = 0.5;  // landmarks / meter
  const double kLandmarkdDistance = 3; // [m]
};

}  // namespace chameleon
