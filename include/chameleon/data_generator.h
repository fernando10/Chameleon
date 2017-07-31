// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <stdint.h>
#include "chameleon/world_generator.h"
#include "chameleon/path_generator.h"
#include "chameleon/odometry_generator.h"
#include "chameleon/observation_generator.h"
#include "chameleon/types.h"
#include "chameleon/math_utils.h"

namespace chameleon
{

class DataGenerator {
public:
  struct DataGeneratorOptions {
    double max_observations = 3;  // maximum observations the robot can make at each timestep
    Eigen::Vector4d odometry_noise =
        (Eigen::Vector4d() << Square(kAlpha1), Square(kAlpha2), Square(kAlpha3), Square(kAlpha4)).finished();
    Eigen::Vector2d measurement_noise = (Eigen::Vector2d() << Square(kBearingStdDev), Square(kRangeStdDev)).finished();
    bool generate_landmarks = true;

    PathGenerator::PathGeneratorOptions path_options;
  };

  DataGenerator(const DataGeneratorOptions& options);

  bool GenerateSimulatedData(SimData* data);

private:
  const DataGeneratorOptions& options_;
  const std::unique_ptr<WorldGenerator> world_generator_;
  const std::unique_ptr<PathGenerator> path_generator_;
  const std::unique_ptr<OdometryGenerator> odometry_generator_;
  const std::unique_ptr<ObservationGenerator> observation_generator_;

  Eigen::Matrix2d measurement_covariance_ = Eigen::Matrix2d::Identity();

  static constexpr double kAlpha1 = 5e-2;
  static constexpr double kAlpha2 = 1e-3;
  static constexpr double kAlpha3 = 5e-2;
  static constexpr double kAlpha4 = 1e-2;

  static constexpr double kBearingStdDev = 0.174533;  // [rad]
  static constexpr double kRangeStdDev = 0.1;  // [m]

};
}  // namespace chameleon