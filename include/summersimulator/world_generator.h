// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include "summersimulator/types.h"
#include "summersimulator/util.h"

namespace summer
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

private:

  double GetTotalDistanceTraveled(const RobotPoseVectorPtr& poses);
  const double kLandmarkDensity = 0.5;  // landmarks / meter
  const double kLandmarkdDistance = 3; // [m]
};

}  // namespace summer
