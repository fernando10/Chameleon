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
  measurement_covariance_ = options_.measurement_noise.asDiagonal();
}

bool DataGenerator::GenerateSimulatedData(SimData* data) {

  // generate a ground truth path (no noise in these poses)
  RobotPoseVectorPtr ground_truth_robot_path = path_generator_->GeneratePath();

  if (ground_truth_robot_path->empty()) {
    LOG(ERROR) << "No poses generated.";
    return false;
  }

  if (options_.generate_landmarks) {
     LandmarkVectorPtr ground_truth_landmarks =  world_generator_->GenerateWorld(ground_truth_robot_path);
     VLOG(1) << fmt::format("generated world with {} landmarks", ground_truth_landmarks->size());
     data->debug.ground_truth_map = ground_truth_landmarks;
     observation_generator_->Reset(ground_truth_landmarks, ground_truth_robot_path);
     observation_generator_->SetCovariance(measurement_covariance_);
  }

  RobotPose noisy_robot_pose = ground_truth_robot_path->at(0);
  RobotPose noise_free_robot_pose = ground_truth_robot_path->at(0);

  data->debug.ground_truth_poses->push_back(noise_free_robot_pose);
  data->debug.noisy_poses->push_back(noisy_robot_pose);

  // load the path into the motion (odometry) generator
  odometry_generator_->SetPath(ground_truth_robot_path);

  // iterate over timesteps, generating odometry and observations
  for (size_t time_step = 0; time_step < ground_truth_robot_path->size() - 1; ++time_step) {

    // first, generate noise-free and noisy odometry
    OdometryMeasurement noise_free_odometry =
        odometry_generator_->GenerateNoiseFreeOdometryMeasurement(time_step);
    RobotPose new_noise_free_pose =
        odometry_generator_->PropagateMeasurement(noise_free_odometry, noise_free_robot_pose);
    noise_free_robot_pose = new_noise_free_pose;

    OdometryMeasurement noisy_odometry =
        odometry_generator_->GenerateNoisyOdometryMeasurement(time_step);
    RobotPose new_noisy_pose =
        odometry_generator_->PropagateMeasurement(noisy_odometry, noisy_robot_pose);
    noisy_robot_pose = new_noisy_pose;

    // then generate noise-free and noisy observations
    RangeFinderObservationVector noise_free_obs = observation_generator_->GenerateObservations(time_step);
    RangeFinderObservationVector noisy_obs = observation_generator_->GenerateNoisyObservations(time_step);

    data->times.push_back(time_step);
    data->debug.ground_truth_poses->push_back(new_noise_free_pose);
    data->debug.noisy_poses->push_back(new_noisy_pose);
    data->debug.noise_free_odometry->push_back(noise_free_odometry);
    data->debug.noisy_odometry->push_back(noisy_odometry);
    data->debug.noise_free_observations.insert({time_step, noise_free_obs});
    data->debug.noisy_observations.insert({time_step, noisy_obs});
  }

  return true;
}
} // namespace chameleon
