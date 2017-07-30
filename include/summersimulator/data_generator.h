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
    double max_observations = 3;  // maximum observations the robot can make at each timestep
    Eigen::Vector4d odometry_noise =
        (Eigen::Vector4d() << Square(kAlpha1), Square(kAlpha2), Square(kAlpha3), Square(kAlpha4)).finished();

    PathGenerator::PathGeneratorOptions path_options;
  };

  DataGenerator(const DataGeneratorOptions& options);

  bool GenerateSimulatedData(SimData* data);

private:
  const DataGeneratorOptions& options_;
  const std::unique_ptr<WorldGenerator> world_generator_;
  const std::unique_ptr<PathGenerator> path_generator_;
  const std::unique_ptr<MotionGenerator> motion_generator_;
  const uint32_t kObservationDim = 3; // range, bearing, id
  const uint32_t kMotionDim = 3; // x, y, theta

  static constexpr double kAlpha1 = 5e-2;
  static constexpr double kAlpha2 = 1e-3;
  static constexpr double kAlpha3 = 5e-2;
  static constexpr double kAlpha4 = 1e-2;
};
}  // namespace summer
