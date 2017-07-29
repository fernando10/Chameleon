// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <stdint.h>
#include "world_generator.h"
#include "motion_generator.h"
#include "types.h"

namespace elninho
{
struct DataGeneratorOptions {
  double num_steps = 100;  // number of steps or keyframes to drop
  RobotPose initial_pose;  // starting position for the robot
  double max_observations = 3;  // maximum observations the robot can make at each timestep
};

struct SimData {
  RangeFinderObservationDeque observations;
  DebugData debug_data;

};

struct DebugData {
  RobotPoseVector ground_truth_poses;
  RobotPoseVector noisy_poses;
  RangeFinderObservationDeque noise_free_observations;
};

class DataGenerator {
public:
  DataGenerator(const DataGeneratorOptions& options);

  SimData GenerateSimulatedData();

private:
  const DataGeneratorOptions& options_;
  const std::unique_ptr<WorldGenerator> world_generator_;
  const std::unique_ptr<PathGenerator> motion_generator_;
  const uint32_t kObservationDim = 3; // range, bearing, id
  const uint32_t kMotionDim = 3; // x, y, theta
};
} // namespace elninho
