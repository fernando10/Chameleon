// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <stdint.h>
#include "summersimulator/world_generator.h"
#include "summersimulator/path_generator.h"
#include "summersimulator/motion_generator.h"
#include "summersimulator/types.h"

namespace summer
{

class DataGenerator {
public:
  struct DataGeneratorOptions {
    size_t num_steps = 100;  // number of steps or keyframes to drop
    RobotPose initial_pose;  // starting position for the robot
    double max_observations = 3;  // maximum observations the robot can make at each timestep

    PathGenerator::PathGeneratorOptions path_options;
  };

  DataGenerator(const DataGeneratorOptions& options);

  SimData GenerateSimulatedData();

private:
  const DataGeneratorOptions& options_;
  const std::unique_ptr<WorldGenerator> world_generator_;
  const std::unique_ptr<PathGenerator> path_generator_;
  const std::unique_ptr<MotionGenerator> motion_generator_;
  const uint32_t kObservationDim = 3; // range, bearing, id
  const uint32_t kMotionDim = 3; // x, y, theta
};
} // namespace elninho
