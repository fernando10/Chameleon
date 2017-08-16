// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include "chameleon/types.h"
#include "chameleon/util.h"

namespace chameleon
{

class WorldGenerator {
 public:
  enum class WorldTypes {
    RectangularField,
    MimicTrajctory
  };

  ///
  /// \brief WorldGenerator
  /// Default Constructor
  WorldGenerator(){}

  LandmarkVectorPtr GenerateWorld(const RobotPoseVectorPtr& robot_poses, WorldTypes type);
  LandmarkVectorPtr GetWorld() const;
  bool RemoveLandmarks(std::vector<uint64_t> lm_ids);


private:

  LandmarkVectorPtr map_;
  double GetTotalDistanceTraveled(const RobotPoseVectorPtr& poses);
//  const double kLandmarkDensity = 0.5;  // landmarks / meter
//  const double kLandmarkdDistance = 3; // [m]
};

}  // namespace chameleon
