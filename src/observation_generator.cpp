// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "chameleon/observation_generator.h"
#include "chameleon/util.h"
#include "glog/logging.h"

namespace chameleon
{
ObservationGenerator::ObservationGenerator() {
  // no data yet, create lists so we don't have any nullptrs
  path_ = std::make_shared<RobotPoseVector>();
  map_ = std::make_shared<LandmarkVector>();
}

ObservationGenerator::ObservationGenerator(const LandmarkVectorPtr map, const RobotPoseVectorPtr path): map_(map), path_(path) {
}

void ObservationGenerator::SetCovariance(Eigen::Matrix2d cov) {
  measurement_covariance_ = cov;
}

void ObservationGenerator::Reset(const LandmarkVectorPtr map, const RobotPoseVectorPtr path) {
  map_ = map;
  path_ = path;
}

RangeFinderObservationVector ObservationGenerator::GenerateObservations(size_t pose_idx) const {
  RangeFinderObservationVector observations;

  if (map_ == nullptr || path_ == nullptr) {
    LOG(ERROR) << "Inalid input to generate observations.";
    return observations;
  }

  // get pose
  RobotPose& robot = path_->at(pose_idx % path_->size());

  // serach for the n landmarks that are within the field of view of this robot (and within range)
  for (const auto& lm_w : *map_) {
    // transfer landmark over to pose frame
    Eigen::Vector2d lm_r = util::DeHomogenizeLandmark<double>(robot.pose.matrix().inverse()
                                                             * util::HomogenizeLandmark<double>(lm_w.vec()));

    if(lm_r.x() < 1e-2) {
      continue;  // landmark too close or behind
    }
    // get angle
    double theta = std::atan2(lm_r.y(), lm_r.x());

    if (std::abs(theta) <= robot.field_of_view/2.) {
      // lm in the field of view, get distance
      double distance = lm_r.norm();
      if (distance <= robot.range) {
        // an observation should be generated
        observations.push_back(RangeFinderObservation(pose_idx, RangeFinderReading(lm_w.id, theta, distance)));
      }
    }
  }
  return observations;
}

RangeFinderObservationVector ObservationGenerator::GenerateNoisyObservations(size_t pose_index) const  {
  RangeFinderObservationVector noise_free_observations = GenerateObservations(pose_index);

  // now add noise to the observation
  MultivariateNormalVariable observation_noise(measurement_covariance_); // zero mean observation noise

  RangeFinderObservationVector noisy_observations;
  for (const auto& obs : noise_free_observations) {
    noisy_observations.push_back(RangeFinderObservation(obs.timestamp, obs.observation + observation_noise()));
  }
  return noisy_observations;
}

}  // namespace chameleon
