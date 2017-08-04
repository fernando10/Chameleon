// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "chameleon/data_generator.h"
#include "chameleon/util.h"
#include "glog/logging.h"

namespace chameleon
{

DataGenerator::DataGenerator(const DataGeneratorOptions& options):
  options_(options){
  InitializeSimulation();
}

void DataGenerator::InitializeSimulation() {
  world_generator_ = util::make_unique<WorldGenerator>();
  path_generator_ = util::make_unique<PathGenerator>(options_.path_options);
  odometry_generator_ = util::make_unique<OdometryGenerator>(options_.odometry_noise);
  observation_generator_ = util::make_unique<ObservationGenerator>();

  measurement_covariance_ = options_.measurement_noise.asDiagonal();
  odometry_generator_->SetPath(path_generator_->GetRobotPath());
  world_generator_->GenerateWorld(path_generator_->GetRobotPath());
  observation_generator_->Reset(world_generator_->GetWorld(), path_generator_->GetRobotPath());
  observation_generator_->SetCovariance(options_.measurement_noise.asDiagonal());
  current_timestep_ = 0;
}

void DataGenerator::Reset() {
  // start from scratch
  InitializeSimulation();
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
    OdometryMeasurement noisy_odometry;
    if (options_.add_noise_to_odometry) {
      noisy_odometry = odometry_generator_->GenerateNoisyOdometryMeasurement(current_timestep_);
    } else {
      noisy_odometry = data->debug.noise_free_odometry;
    }
    data->debug.noisy_odometry = noisy_odometry;

    // odometry data which would be sensed (to be used in the optimization)
    data->odometry = noisy_odometry;

    // propagate to get new integrated pose
    noisy_robot = odometry_generator_->PropagateMeasurement(noisy_odometry);
  }

  // generate some observations (with and without noise)
  RangeFinderObservationVector noise_free_obs =  observation_generator_->GenerateObservations(current_timestep_);
  RangeFinderObservationVector noisy_obs = observation_generator_->GenerateNoisyObservations(current_timestep_);

  data->debug.noisy_pose = noisy_robot;
  data->debug.ground_truth_map = world_generator_->GetWorld(); // this always points to the same data
  data->debug.noise_free_observations = noise_free_obs;
  data->debug.noisy_observations = noisy_obs;
  data->timestamp = current_timestep_;
  data->observations = noisy_obs;
  current_timestep_++;
  return true;
}

} // namespace chameleon
