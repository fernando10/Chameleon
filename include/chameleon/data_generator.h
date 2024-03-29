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
    bool generate_landmarks = true;
    bool add_noise_to_odometry = true;
    double prob_missed_detection = 0.0; // probability of not detecting a landmark that should be seen
    double prob_false_positive = 0.0; // probability of detecting a landmarks that is not there
    std::vector<uint64_t> remove_lm_ids;
    std::vector<uint64_t> change_lm_ids;
    PathGenerator::PathGeneratorOptions path_options;
  };

  DataGenerator(const DataGeneratorOptions& options);

  bool GetData(RobotData* const data) override;
  LandmarkVectorPtr GetNoisyMap();
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

//    static constexpr double kBearingStdDev = 0.01;  // [rad] ~10 degrees
//    static constexpr double kRangeStdDev = 0.005;  // [m]

};
}  // namespace chameleon
