// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "chameleon/data_generator.h"
#include "chameleon/util.h"
#include "glog/logging.h"

namespace chameleon
{

DataGenerator::DataGenerator(const DataGeneratorOptions& options):
  options_(options),
  world_generator_(util::make_unique<WorldGenerator>()),
  path_generator_(util::make_unique<PathGenerator>(options.path_options)),
  odometry_generator_(util::make_unique<OdometryGenerator>(options.odometry_noise)),
  observation_generator_(util::make_unique<ObservationGenerator>()) {

  // build up the sim poses/landmarks
  measurement_covariance_ = options_.measurement_noise.asDiagonal();
  odometry_generator_->SetPath(path_generator_->GetRobotPath());
  world_generator_->GenerateWorld(path_generator_->GetRobotPath());
}

bool DataGenerator::GetRobotData(RobotData* const data) {
  // since we're simulating should always return something here
  data->debug.ground_truth_pose = path_generator_->GetRobot(current_timestep_);

  RobotPose noisy_robot;
  if (current_timestep_ == 0) {
    // first timestamp, real and noisy robot poses are the same (no movement yet)
    noisy_robot = data->debug.ground_truth_pose;
  } else {
    // robot has moved, generate pose by integrating noisy odometry

    // get the ground truth odometry between the current pose and the previous
    data->debug.noise_free_odometry =
        odometry_generator_->GenerateNoiseFreeOdometryMeasurement(current_timestep_);

    // add noise to odometry measurement
    OdometryMeasurement noisy_odometry =
        odometry_generator_->GenerateNoisyOdometryMeasurement(current_timestep_);
    data->debug.noisy_odometry = noisy_odometry;

    // propagate to get new integrated pose
    noisy_robot = odometry_generator_->PropagateMeasurement(noisy_odometry);
  }

  data->debug.noisy_pose = noisy_robot;
  data->debug.ground_truth_map = world_generator_->GetWorld(); // this always points to the same data
  data->timestamp = current_timestep_;
  current_timestep_++;
  return true;
}

} // namespace chameleon
