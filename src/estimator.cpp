// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "chameleon/estimator.h"
#include <memory>
#include "chameleon/ceres_cost_terms.h"
#include "chameleon/odometry_generator.h"  // here just for the odometry propagation, should prob. move elsewhere
namespace chameleon
{
namespace ceres
{

Estimator::Estimator(const EstimatorOptions& options): options_(options) {
  Reset();
}


void Estimator::Reset() {
  VLOG(1) << "Resetting estimator.";
  ::ceres::Problem::Options ceres_problem_options;
  ceres_problem_options.loss_function_ownership = ::ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres_problem_options.local_parameterization_ownership = ::ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres_problem_ = util::make_unique<::ceres::Problem>(ceres_problem_options);
  local_param_ = std::unique_ptr<::ceres::LocalParameterization>(
                   new ::ceres::AutoDiffLocalParameterization<Sophus::chameleon::AutoDiffLocalParamSE2, Sophus::SE2d::num_parameters,
                   Sophus::SE2d::DoF>);
  if (options_.huber_loss_a > 0){
    ceres_loss_function_ = util::make_unique<::ceres::HuberLoss>(options_.huber_loss_a);
  }
  landmarks_.clear();
  states_.clear();
  state_2_landmark_multimap_.clear();
  landmark_2_state_multimap_.clear();
}

void Estimator::AddData(const RobotData &data) {
  VLOG(2) << "Adding new data to problem";

  // Create a new State
  StatePtr new_state = std::make_shared<State>();
  new_state->timestamp = data.timestamp;

  // iterator pointing to the last state
  StatePtrMap::reverse_iterator last_state_rit = states_.rbegin();

  // initialize state by propagating odometry
  if (states_.empty()) {
    VLOG(1) << "First state received.";
    // this is the very first state, use identity for now
    // TODO: Get initial pose externally so it's not arbitrarily fixed at the origin
    new_state->robot.pose = Sophus::SE2d(0, Eigen::Vector2d::Zero());
  } else {
    // use odometry to propagate previous measurement forward and get estimated state
    RobotPose guess = OdometryGenerator::PropagateMeasurement(data.odometry, last_state_rit->second->robot);
    VLOG(2) <<  fmt::format("Propagated pose (id:{}) at: {} and got pose at: {}",last_state_rit->first, last_state_rit->second->robot, guess);
    new_state->robot.pose = guess.pose;
  }

  ceres_problem_->AddParameterBlock(new_state->data(), Sophus::SE2d::num_parameters);
  ceres_problem_->SetParameterization(new_state->data(), local_param_.get());

  // Add state to problem
  uint64_t id = GetNewStateId();
  new_state->id = id;
  states_.insert({id, new_state});
  last_optimized_state_ = new_state;
  VLOG(3) << fmt::format("Added state with id: {} to problem.", id);
  //last_state_rit--; // move so it points to the newly inserted state;
  VLOG(1) << fmt::format(" last state rit points to state id: {}", last_state_rit->first);

  if(states_.size() == 1) {
    VLOG(3) << fmt::format("Setting parameter block id:{} constant.", new_state->id);
    ceres_problem_->SetParameterBlockConstant(new_state->data());  // fix first pose to take care of nullspaces
  }

  ///////////////////////////////////
  ////// OBSERVATION FACTORS
  /// ///////////////////////////////
  if (!data.observations.empty() && options_.add_observations) {
    DataAssociationMap association =  DataAssociation::AssociateDataToLandmarks(data.observations, landmarks_,
                                                                                DataAssociation::DataAssociationType::Known);
    // create new landmarks, if necessary
    // - add new landmarks to problem
    // - create factor between current state and newly added landmark
    CreateNewLandmarks(association, data.observations, id);

    // add factors between this state and exisitng landmakrs that have been associated to measurements
    for (const auto& e : association) {
      // association is: index in observation vector -> landmark_id
      CreateObservationFactor(id, e.second, data.observations.at(e.first));
    }
  }

  ////////////////////////////////
  ////// ODOMETRY FACTOR
  /// ////////////////////////////
  if (states_.size() > 1) {
    VLOG(1) << fmt::format(" std::next(last_state_rit, 1) points to state id: {}", std::next(last_state_rit, 1)->first);

    // only add odometry if this is not the very first state
    CreateOdometryFactor(std::next(last_state_rit, 1)->first, last_state_rit->first, data.odometry);
  }
  VLOG(2) << "Finished adding data.";
}

void Estimator::Solve() {
  if (states_.size() >= options_.min_states_for_solve) {
    VLOG(1) << "Solving full SLAM problem.";
    ::ceres::Solve(options_.ceres_options , ceres_problem_.get(), &summary_);

    if (options_.print_full_summary) {
      LOG(INFO) << summary_.FullReport();
    }else if (options_.print_brief_summary) {
      LOG(INFO) << summary_.BriefReport();
    }
  } else {
    VLOG(1) << fmt::format("{} states in estimator, waititng for {} states to run optimizatoin.",
                           states_.size(), options_.min_states_for_solve);
  }
}

uint64_t Estimator::GetNewStateId() {
  return states_.size() + 1;
}

uint64_t Estimator::GetNewLandmarkId() {
  return landmarks_.size() +1;
}

void Estimator::CreateNewLandmarks(const DataAssociationMap& data_association,
                                   const RangeFinderObservationVector& observations, uint64_t state_id) {
  VLOG(2) << fmt::format("Creating new landmakrs from state {}", state_id);
  for (size_t meas_idx = 0; meas_idx < observations.size(); ++ meas_idx) {
    uint64_t lm_id = 0;

    if (data_association.find(meas_idx) == data_association.end()) {
      // not found, create new landmark -- the assumption here is that there will not be two measurements for the
      // same landmark
      VLOG(2) << fmt::format("No data association for observation idx: {}, creating a new landmark and adding to map", meas_idx);
      lm_id = CreateLandmark(observations.at(meas_idx), state_id);

      // add residual
      CreateObservationFactor(state_id, lm_id, observations.at(meas_idx));

    } else if (landmarks_.find(data_association.at(meas_idx)) == landmarks_.end() &&
               options_.data_association_strategy == DataAssociation::DataAssociationType::Known) {
      // if the measurement is associated to a landmark id that does not exist, but we have known data associations
      // create a landmark with the given id, the factor will be created later
      VLOG(2) << fmt::format("Observation idx: {} has a data association but the landmark id ({}) is not in map. Creating landmark.",
                             meas_idx, data_association.at(meas_idx));

      lm_id = data_association.at(meas_idx);
      CreateLandmark(observations.at(meas_idx), state_id, lm_id);
    } else if (landmarks_.find(data_association.at(meas_idx)) == landmarks_.end()) {
      LOG(ERROR) << fmt::format("Observation idx: {} is associated to landmark id: {}, however no such landmark exists in the map. Should not happen",
                                meas_idx, data_association.at(meas_idx));
      continue;
    } else {
      VLOG(2) << fmt::format("Observation idx: {} is associated to landmark id {}, nothing to add to map.",
                             meas_idx, data_association.at(meas_idx));
    }

    //update bookeeping
    landmark_2_state_multimap_.insert({lm_id, state_id});
    state_2_landmark_multimap_.insert({state_id, lm_id});
  }
}

bool Estimator::CheckStateExists(uint64_t state_id) {
  return states_.find(state_id) != states_.end();
}

bool Estimator::CheckLandmarkExists(uint64_t lm_id) {
  return landmarks_.find(lm_id) != landmarks_.end();
}

void Estimator::CreateObservationFactor(const uint64_t state_id,
                                        const uint64_t landmark_id, const RangeFinderObservation &obs) {
  VLOG(2) << fmt::format("Trying to create observation factor between state {} and landmark {}", state_id, landmark_id);

  if (!CheckStateExists(state_id)) {
    LOG(ERROR) << fmt::format("Error adding observation factor, state id {} requested but not in state map", state_id);
    return;
  }
  if (!CheckLandmarkExists(landmark_id)) {
    LOG(ERROR) << fmt::format("Error adding observation factor, landmark id {} requested but not in landmark map", landmark_id);
    return;
  }

  // add observation factor between state and landmark to problem
  //TODO: Get correct measurement covariance
  RangeFinderCovariance range_cov = RangeFinderCovariance::Identity();
  range_cov(RangeFinderReading::kIndexBearing, RangeFinderReading::kIndexBearing) = Deg2Rad(10); // 10 degrees
  range_cov(RangeFinderReading::kIndexRange, RangeFinderReading::kIndexRange) = 0.1; // 10cm
  Eigen::LLT<RangeFinderCovariance> llt_of_information(range_cov.inverse());
  RangeFinderCovariance sqrt_information = llt_of_information.matrixL().transpose();

  auto observation_cost_function = new ::ceres::AutoDiffCostFunction<ceres::RangeFinderObservationCostFunction,
      RangeFinderReading::kMeasurementDim, Sophus::SE2d::num_parameters, Landmark::kLandmarkDim>(
        new ceres::RangeFinderObservationCostFunction (obs, sqrt_information));

  ceres_problem_->AddResidualBlock(observation_cost_function, ceres_loss_function_.get(),
                                   states_.at(state_id)->data(), landmarks_.at(landmark_id)->data());
  VLOG(2) << fmt::format("Created observation factor between state {} and landmark {}", state_id, landmark_id);

}

void Estimator::CreateOdometryFactor(const uint64_t prev_state_id, const uint64_t current_state_id,
                                     const OdometryMeasurement& odometry) {
  VLOG(2) << fmt::format("Trying to create odometry factor between state {} and state {}", prev_state_id, current_state_id);
  if (!CheckStateExists(prev_state_id)) {
    LOG(ERROR) << fmt::format("Error adding odometry factor, state id {} requested but not in state map", prev_state_id);
    return;
  }
  if (!CheckStateExists(current_state_id)) {
    LOG(ERROR) << fmt::format("Error adding odometry factor, state id {} requested but not in state map", current_state_id);
    return;
  }
  // add observation factor between state and landmark to problem
  //TODO: Get correct odometry covariance
  OdometryCovariance odometry_cov = OdometryCovariance::Identity();
  odometry_cov(0,0) = 5e-1;
  odometry_cov(1,1) = 1e-1;
  odometry_cov(2,2) = 5e-1;
  OdometryCovariance inv_cov = odometry_cov.inverse();
  Eigen::LLT<OdometryCovariance> llt_of_information(inv_cov);
  OdometryCovariance sqrt_information = llt_of_information.matrixL().transpose();

  auto odometry_cost_function = new ::ceres::AutoDiffCostFunction<ceres::OdometryCostFunction,
      OdometryMeasurement::kMeasurementDim, Sophus::SE2d::num_parameters, Sophus::SE2d::num_parameters>(
        new ceres::OdometryCostFunction (odometry, sqrt_information));

  ceres_problem_->AddResidualBlock(odometry_cost_function, ceres_loss_function_.get(),
                                   states_.at(prev_state_id)->data(),  states_.at(current_state_id)->data());
  VLOG(2) << fmt::format("Created odometry factor between state {} and state {}",
                         prev_state_id, current_state_id);

}


// TODO: remove duplicated code
void Estimator::CreateLandmark(const RangeFinderObservation &obs, uint64_t state_id, uint64_t lm_id) {
  VLOG(2) << fmt::format("Trying to create landmark observed from state id: {}, landmark id was given: {}", state_id, lm_id);
  if (!CheckStateExists(state_id)) {
    LOG(ERROR) << fmt::format("Error creating new landmark, state id {} requested but not in states map.", state_id);
    return;
  }

  StatePtr& state = states_.at(state_id);
  // get the landmark in the robot reference frame
  Landmark lm_r(obs.observation.range * std::cos(obs.observation.theta),
                obs.observation.range * std::sin(obs.observation.theta));
  // transfer the landmark over to the world frame
  Landmark lm_w = state->robot * lm_r;
  landmarks_.insert({lm_id, std::make_shared<Landmark>(lm_w)});
  VLOG(2) << fmt::format("Created landmark observed from state id: {}, lm id: {}", state_id, lm_id);
}

uint64_t Estimator::CreateLandmark(const RangeFinderObservation &obs, uint64_t state_id) {
  VLOG(2) << fmt::format("Trying to create landmark observed from state id: {}", state_id);

  if (!CheckStateExists(state_id)) {
    LOG(ERROR) << fmt::format("Error creating new landmark, state id {} requested but not in states map.", state_id);
    return 0;
  }

  const StatePtr& state = states_.at(state_id);
  // get the landmark in the robot reference frame
  Landmark lm_r(obs.observation.range * std::cos(obs.observation.theta),
                obs.observation.range * std::sin(obs.observation.theta));
  // transfer the landmark over to the world frame
  Landmark lm_w = state->robot * lm_r;

  uint64_t lm_id = GetNewLandmarkId();
  landmarks_.insert({lm_id, std::make_shared<Landmark>(lm_w)});
  VLOG(2) << fmt::format("Created landmark observed from state id: {}, lm id: {}", state_id, lm_id);
  return lm_id;
}

void Estimator::GetEstimationResult(EstimatedData* data) const {
  if(data != nullptr) {
    data->landmarks = landmarks_;
    data->states = states_;  //TODO: dont copy entire map of pointers over every time
  }
}

}  // namespace ceres
}  // namespace chameleon
