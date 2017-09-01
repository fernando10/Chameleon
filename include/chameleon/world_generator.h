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
  LandmarkVectorPtr SampleWorld();
  bool RemoveLandmarks(std::vector<uint64_t> lm_ids);
  void ChangeLandmarks(std::vector<uint64_t> lm_ids);


private:

  LandmarkVectorPtr map_;
  LandmarkVectorPtr noisy_map_;
  double GetTotalDistanceTraveled(const RobotPoseVectorPtr& poses);
  const double kMapSigma = 0.05;
};

}  // namespace chameleon
