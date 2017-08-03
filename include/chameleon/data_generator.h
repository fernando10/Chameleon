// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <stdint.h>
#include "chameleon/data_provider_base.h"
#include "chameleon/world_generator.h"
#include "chameleon/path_generator.h"
#include "chameleon/odometry_generator.h"
#include "chameleon/observation_generator.h"
#include "chameleon/types.h"
#include "chameleon/math_utils.h"

namespace chameleon
{

class DataGenerator : public DataProviderBase {
public:
  struct DataGeneratorOptions {
    Eigen::Vector4d odometry_noise =
        (Eigen::Vector4d() << Square(kAlpha1), Square(kAlpha2), Square(kAlpha3), Square(kAlpha4)).finished();
    Eigen::Vector2d measurement_noise = (Eigen::Vector2d() << Square(kBearingStdDev), Square(kRangeStdDev)).finished();
    bool generate_landmarks = true;
    bool add_noise_to_odometry = true;
    PathGenerator::PathGeneratorOptions path_options;
  };

  DataGenerator(const DataGeneratorOptions& options);

  bool GetRobotData(RobotData* const data) override;
  void Reset();

private:
  bool GenerateSimulatedData(SimData* data);
  void InitializeSimulation();

  const DataGeneratorOptions& options_;
  std::unique_ptr<WorldGenerator> world_generator_;
  std::unique_ptr<PathGenerator> path_generator_;
  std::unique_ptr<OdometryGenerator> odometry_generator_;
  std::unique_ptr<ObservationGenerator> observation_generator_;

  Eigen::Matrix2d measurement_covariance_ = Eigen::Matrix2d::Identity();

  double current_timestep_ = 0;

  static constexpr double kAlpha1 = 5e-2;
  static constexpr double kAlpha2 = 1e-3;
  static constexpr double kAlpha3 = 5e-2;
  static constexpr double kAlpha4 = 1e-2;

//  static constexpr double kBearingStdDev = 0.174533;  // [rad] ~10 degrees
//  static constexpr double kRangeStdDev = 0.1;  // [m]
  static constexpr double kBearingStdDev = 0.087;  // [rad] ~10 degrees
  static constexpr double kRangeStdDev = 0.05;  // [m]

};
}  // namespace chameleon
