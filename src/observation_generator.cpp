// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "chameleon/observation_generator.h"
#include "chameleon/util.h"

namespace chameleon
{

ObservationGenerator::ObservationGenerator(const LandmarkVectorPtr map, const RobotPoseVectorPtr path): map_(map), path_(path) {
}

void ObservationGenerator::Reset(const LandmarkVectorPtr map, const RobotPoseVectorPtr path) {
  map_ = map;
  path_ = path;
}

RangeFinderObservationVector ObservationGenerator::GenerateObservations(size_t pose_idx) {
  RangeFinderObservationVector observations;

  if (path_ == nullptr || path_->size() < pose_idx || pose_idx < 0) {
    LOG(ERROR) << "Inalid input to generate observations.";
    return observations;
  }

  // get pose
  RobotPose& robot = path_->at(pose_idx);

  // serach for the n landmarks that are within the field of view of this robot (and within range)
  for (const auto& lm_w : *map_) {
    // transfer landmark over to pose frame
    Eigen::Vector2d lm_r= util::DeHomogenizeLandmark(robot.pose.matrix().inverse() * util::HomogenizeLandmark(lm_w.vec()));

    if(lm_r.x() < 1e-2) {
      continue;  // landmark too close or behind
    }
    // get angle
    double theta = std::atan2(lm_r.y(), lm_r.x());

    if (abs(theta) <= robot.field_of_view/2.) {
      // lm in the field of view, get distance
      double distance = lm_r.norm();
      if (distance <= robot.range) {
        // an observation should be generated
        observations.push_back(RangeFinderObservation(pose_idx, RangeFinderReading(theta, distance)));
      }
    }
  }
  return observations;
}

}  // namespace chameleon
