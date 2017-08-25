// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "chameleon/estimator.h"
#include <memory>
#include <numeric>
#include <fstream>

#include "chameleon/ceres_cost_terms.h"
#include "chameleon/id_provider.h"
#include "chameleon/odometry_generator.h"  // here just for the odometry propagation, should prob. move elsewhere
#include "chameleon/persistence_filter_utils.h"

namespace chameleon
{
namespace ceres
{

double lambda_u = 1e1;

double lambda_l = 1e-3;
std::function<double(double)> logS_T = std::bind(log_general_purpose_survival_function, std::placeholders::_1, lambda_l, lambda_u);


Estimator::Estimator(const EstimatorOptions& options): options_(options) {
  Reset();
}

////////////////////////////////////////////////////////////////
/// \brief Estimator::Reset
////////////////////////////////////////////////////////////////
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
  persistence_filter_map_.clear();
  state_2_landmark_multimap_.clear();
  landmark_2_state_multimap_.clear();
  localization_mode_ = false;
  latest_timestamp_ = 0;
  last_state_id_ = 0;
  data_assoc_results_.Clear();
}

////////////////////////////////////////////////////////////////
/// \brief Estimator::SetMap
////////////////////////////////////////////////////////////////
void Estimator::SetMap(LandmarkVectorPtr prior_map) {
  // map provided
  landmarks_.clear();
  for (Landmark& lm : *prior_map) {
    LandmarkPtr lm_ptr = std::make_shared<Landmark>();
    *lm_ptr = lm;
    landmarks_[lm.id] = lm_ptr;

    // add the parameter blocks now so we can set them constant
    ceres_problem_->AddParameterBlock(lm_ptr->data(), Landmark::kLandmarkDim);
    ceres_problem_->SetParameterBlockConstant(lm_ptr->data());
  }

  // TEMP FOR DEMO
  persistence_filter_graph_[16][17] = 2;
  persistence_filter_graph_[17][18] = 3.5;
  persistence_filter_graph_[19][18] = 3.5;
  persistence_filter_graph_[20][19] = 2;


  localization_mode_ = true;
}

////////////////////////////////////////////////////////////////
/// \brief Estimator::AddData
////////////////////////////////////////////////////////////////
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
  VLOG(3) << fmt::format("Added state with id: {} to problem.", id);

  if(states_.size() == 1) {
    VLOG(3) << fmt::format("Setting parameter block id:{} constant.", new_state->id);
    ceres_problem_->SetParameterBlockConstant(new_state->data());  // fix first pose to take care of nullspaces
    new_state->fixed = true;
  }

  ///////////////////////////////////
  ////// OBSERVATION FACTORS
  if (!data.observations.empty() && options_.add_observations) {

    // get  a map with obs_index -> landmark_id
    LandmarkPtrMap visible_landmarks = GetLandmarksThatShouldBeVisible(new_state->robot);
    Marginals marginals;
    if (states_.size() > options_.min_states_for_solve && options_.data_association_strategy !=
        DataAssociation::DataAssociationType::Known) {
      // try to get covariances for the state variables we care about for this data association
      GetMarginals(last_state_id_, visible_landmarks, &marginals);
    }else {
      marginals.robot = new_state->robot;
      marginals.landmarks = visible_landmarks;
    }
    DataAssociationMap association = DataAssociation::AssociateDataToLandmarks(data.observations
                                                                               ,visible_landmarks
                                                                               ,marginals
                                                                               ,options_.data_association_strategy);

    // save association results for visualizing
    data_assoc_results_.observations = data.observations;
    data_assoc_results_.associations = association;

    // check if any landmarks that should have been observed were not so we can update the belief over that landmark
    for (const auto& lm : visible_landmarks) {
      bool found = false;
      for (const auto& a : association) {
        if (a.second == lm.first) {
          found = true;
          break;
        }
      }
      if (!found) {
        VLOG(1) << fmt::format("Landmark id {} should have been observed but was not.", lm.first);
        if (persistence_filter_map_.find(lm.first) != persistence_filter_map_.end()) {
          persistence_filter_map_.at(lm.first)->update(false, data.timestamp+1, options_.filter_options.P_M,
                                                       options_.filter_options.P_F);
        }
      }
    }

    // create new landmarks, if necessary
    if (!localization_mode_) {
      CreateNewLandmarks(association, data.observations, id);
    }

    // add factors between this state and landmakrs that have been associated to measurements
    CheckAndAddObservationFactors(id, association, data.observations);
  }

  ////////////////////////////////
  ////// ODOMETRY FACTOR
  if (states_.size() > 1) {
    // only add odometry if this is not the very first state
    CreateOdometryFactor(std::next(last_state_rit, 1)->first, last_state_rit->first, data.odometry);
  }

  last_state_id_ = id;
  latest_timestamp_ = data.timestamp;
  VLOG(2) << "Finished adding data.";
}

////////////////////////////////////////////////////////////////
/// \brief Estimator::GetLandmarksThatShouldBeVisible
////////////////////////////////////////////////////////////////
LandmarkPtrMap Estimator::GetLandmarksThatShouldBeVisible(const RobotPose& robot) {
  LandmarkPtrMap visible_landmarks;

  for (const auto& e : landmarks_) {

    // transfer landmark over to pose frame
    Eigen::Vector2d lm_r = util::DeHomogenizeLandmark<double>(robot.pose.matrix().inverse()
                                                              * util::HomogenizeLandmark<double>(e.second->vec()));

    if(lm_r.x() < 1e-2) {
      continue;  // landmark too close or behind
    }
    // get angle
    double theta = AngleWraparound<double>(std::atan2(lm_r.y(), lm_r.x()));

    if (std::abs(theta) <= robot.field_of_view/2.) {
      // lm in the field of view, get distance
      double distance = lm_r.norm();
      if (distance <= robot.range) {
        // should be visible
        visible_landmarks.insert({e.first, e.second});
      }
    }
  }

  return visible_landmarks;
}

////////////////////////////////////////////////////////////////
/// \brief Estimator::Solve
////////////////////////////////////////////////////////////////
void Estimator::Solve() {
  if (states_.size() >= options_.min_states_for_solve) {
    VLOG(1) << "Solving full SLAM problem.";
    ::ceres::Solve(options_.ceres_options , ceres_problem_.get(), &summary_);

    if (options_.compute_landmark_covariance) {
      GetMapUncertainty();
    }

    if (options_.compute_latest_pose_covariance) {
      GetLatestPoseUncertainty();
    }

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


////////////////////////////////////////////////////////////////
/// \brief Estimator::GetMarginals
////////////////////////////////////////////////////////////////
bool Estimator::GetMarginals(uint64_t state_id, LandmarkPtrMap lms, Marginals* res) {
  std::vector<uint64_t> lm_ids;
  for(const auto& e : lms) {
    lm_ids.push_back(e.first);
  }
  return GetMarginals(state_id, lm_ids, res);
}

bool Estimator::GetMarginals(uint64_t state_id, std::vector<uint64_t> lm_ids, Marginals* res) {

  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  if(!CheckStateExists(last_state_id_)) {
    LOG(ERROR) << "Trying to get marginal covariance for state id: " << last_state_id_ << " but it's not in the problem";
    return false;
  }

  StatePtr state = states_.at(state_id);
  covariance_blocks.push_back(std::make_pair(state->data(), state->data()));


  for (const auto& landmark_id : lm_ids) {
    if (!CheckLandmarkExists(landmark_id)) {
      LOG(ERROR) << fmt::format("Covariance for lm id: {} requested but that id is not in the map.", landmark_id);
      return false;
    }
    LandmarkPtr lm = landmarks_.at(landmark_id);
    if (unitialized_landmarks_.find(landmark_id) != unitialized_landmarks_.end() ||
        !lm->active) {
      // landmark hasn't been added to the estimation yet or landmark is fixed
      continue;
    }
    covariance_blocks.push_back(std::make_pair(lm->data(), lm->data())); // get the marginal covariance
    covariance_blocks.push_back(std::make_pair(state->data(), lm->data())); // and the pose-landmark covariance
  }


  ::ceres::Covariance::Options options;
  ::ceres::Covariance covariance(options);
  Eigen::MatrixXd cov_out(State::kStateDim + lm_ids.size() * Landmark::kLandmarkDim,
                          State::kStateDim + lm_ids.size() * Landmark::kLandmarkDim);

  bool success = covariance.Compute(covariance_blocks, ceres_problem_.get());

  if (success) {
    Eigen::Map<Eigen::Matrix<double,State::kStateDim, State::kStateDim, Eigen::RowMajor>> state_cov(
          cov_out.block<State::kStateDim, State::kStateDim>(0, 0).data());

    covariance.GetCovarianceBlockInTangentSpace(state->data(), state->data(), state_cov.data());
    res->covariances.insert({std::make_pair(state_id, state_id), state_cov});
    res->robot = state->robot;
    res->pose_id = state_id;

    // get the covariance blocks for all the requested landmarks
    for (size_t idx = 0; idx < lm_ids.size(); ++idx) {

      if (unitialized_landmarks_.find(lm_ids.at(idx)) != unitialized_landmarks_.end()) {
        // landmark hasn't been added to the estimation yet...
        res->covariances.insert({std::make_pair(lm_ids.at(idx), lm_ids.at(idx)), LandmarkCovariance::Identity()});
        res->covariances.insert({std::make_pair(state_id, lm_ids.at(idx)), Eigen::Matrix<double, 3, 2>::Zero()});
        continue;
      }

      if (!landmarks_.at(lm_ids.at(idx))->active) {
        res->covariances.insert({std::make_pair(lm_ids.at(idx), lm_ids.at(idx)), landmarks_.at(lm_ids.at(idx))->covariance });
        res->covariances.insert({std::make_pair(state_id, lm_ids.at(idx)), Eigen::Matrix<double, 3, 2>::Zero()});  // TODO: how to compute state-marginal covariance block when map was given?
        continue;
      }

      // first get the diagonal block
      Eigen::Map<Eigen::Matrix<double,Landmark::kLandmarkDim, Landmark::kLandmarkDim, Eigen::RowMajor>> lm_cov(
            cov_out.block<Landmark::kLandmarkDim, Landmark::kLandmarkDim>(
              State::kStateDim + idx * Landmark::kLandmarkDim,
              State::kStateDim + idx * Landmark::kLandmarkDim).data());
      LandmarkPtr lm = landmarks_.at(lm_ids.at(idx));
      covariance.GetCovarianceBlock(lm->data(), lm->data(), lm_cov.data());
      res->covariances.insert({std::make_pair(lm_ids.at(idx), lm_ids.at(idx)), lm_cov});
      res->landmarks[lm_ids.at(idx)] = lm;

      // now get the off-diagonal pose-landmark covariance
      Eigen::Map<Eigen::Matrix<double,State::kStateDim, Landmark::kLandmarkDim, Eigen::RowMajor>> state_lm_cov(
            cov_out.block<State::kStateDim, Landmark::kLandmarkDim>(0, State::kStateDim + idx * Landmark::kLandmarkDim).data());
      covariance.GetCovarianceBlock(state->data(), lm->data(), state_lm_cov.data());
      res->covariances.insert({std::make_pair(state_id, lm_ids.at(idx)), state_lm_cov});

      // also fill out the landmark-pose
      cov_out.block<Landmark::kLandmarkDim, State::kStateDim>(State::kStateDim + idx * Landmark::kLandmarkDim, 0) =
          state_lm_cov.transpose();
    }
  }else {
    LOG(ERROR) << "Unable to compute marginal covariances.";
  }

  return success;

}


////////////////////////////////////////////////////////////////
/// \brief Estimator::GetLatestPoseUncertainty
////////////////////////////////////////////////////////////////
bool Estimator::GetLatestPoseUncertainty() {

  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  if(!CheckStateExists(last_state_id_)) {
    LOG(ERROR) << "Trying to get marginal covariance for state id: " << last_state_id_ << " but it's not in the problem";
    return false;
  }

  StatePtr state = states_.at(last_state_id_);
  covariance_blocks.push_back(std::make_pair(state->data(), state->data()));

  ::ceres::Covariance::Options options;
  ::ceres::Covariance covariance(options);

  Eigen::MatrixXd cov_out;
  cov_out.resize(covariance_blocks.size() * Sophus::SE2d::DoF, covariance_blocks.size() * Sophus::SE2d::DoF);

  bool success = covariance.Compute(covariance_blocks, ceres_problem_.get());
  if (success) {
    covariance.GetCovarianceBlockInTangentSpace(state->data(), state->data(), cov_out.data());
    state->robot.covariance = cov_out;  // update the covariance in the state
    VLOG(1) << "Latest state covariance: \n" << cov_out;
  }
  return success;

}

////////////////////////////////////////////////////////////////
/// \brief Estimator::GetMapUncertainty
////////////////////////////////////////////////////////////////
void Estimator::GetMapUncertainty() {
  std::vector<uint64_t> lm_ids;
  for (const auto& pair : landmarks_) {
    if (!pair.second->active) {
      continue;
    }
    lm_ids.push_back(pair.first);
  }

  Eigen::MatrixXd lm_cov;
  if (GetLandmarkUncertainty(lm_ids, &lm_cov)) {
    VLOG(2) << " Got uncertainty for "  << lm_ids.size() << " landmarks";
  }
}

////////////////////////////////////////////////////////////////
/// \brief Estimator::GetLandmarkUncertainty
////////////////////////////////////////////////////////////////
bool Estimator::GetLandmarkUncertainty(std::vector<uint64_t> landmark_ids, Eigen::MatrixXd* cov_out) {

  // setup vector specifying which blocks of the covariance matrix we want to recover
  // we just recover the marginal covariance here as we are not interested in the joint for now
  std::vector<std::pair<const double*, const double*>> covariance_blocks;

  for (const auto& landmark_id : landmark_ids) {
    if (!CheckLandmarkExists(landmark_id)) {
      LOG(ERROR) << fmt::format("Covariance for lm id: {} requested but that id is not in the map.", landmark_id);
      return false;
    }
    LandmarkPtr lm = landmarks_.at(landmark_id);
    covariance_blocks.push_back(std::make_pair(lm->data(), lm->data()));  // tell ceres what block of the covariance matrix we want
  }

  ::ceres::Covariance::Options options;
  //options.algorithm_type = ::ceres::CovarianceAlgorithmType::SUITE_SPARSE_QR;
  //options.min_reciprocal_condition_number = 1e-14;
  ::ceres::Covariance covariance(options);

  cov_out->resize(covariance_blocks.size() * Landmark::kLandmarkDim, covariance_blocks.size() * Landmark::kLandmarkDim);

  bool success = covariance.Compute(covariance_blocks, ceres_problem_.get());
  if (success) {
    // get the covariance blocks for all the requested landmarks
    for (size_t idx = 0; idx < landmark_ids.size(); ++idx) {
      Eigen::Map<Eigen::Matrix<double,Landmark::kLandmarkDim, Landmark::kLandmarkDim, Eigen::RowMajor>> lm_cov(
            cov_out->block<Landmark::kLandmarkDim, Landmark::kLandmarkDim>(
              idx * Landmark::kLandmarkDim,
              idx * Landmark::kLandmarkDim).data());
      LandmarkPtr lm = landmarks_.at(landmark_ids.at(idx));
      covariance.GetCovarianceBlock(lm->data(), lm->data(), lm_cov.data());
      lm->covariance = lm_cov;  // update the covariance in the landmark
    }
  }
  return success;
}

////////////////////////////////////////////////////////////////
/// \brief Estimator::GetNewStateId
////////////////////////////////////////////////////////////////
uint64_t Estimator::GetNewStateId() {
  return  IdGenerator::Instance::NewId();
}

////////////////////////////////////////////////////////////////
/// \brief Estimator::GetNewLandmarkId
////////////////////////////////////////////////////////////////
uint64_t Estimator::GetNewLandmarkId() {
  return IdGenerator::Instance::NewId();
}

////////////////////////////////////////////////////////////////
/// \brief Estimator::CreateNewLandmarks
////////////////////////////////////////////////////////////////
void Estimator::CreateNewLandmarks(DataAssociationMap& data_association,
                                   const RangeFinderObservationVector& observations, uint64_t state_id) {

  VLOG(2) << fmt::format("Creating new landmakrs from state {}", state_id);

  for (size_t meas_idx = 0; meas_idx < observations.size(); ++ meas_idx) {
    uint64_t lm_id = 0;

    if (data_association.find(meas_idx) == data_association.end()) {
      // not found, create new landmark -- the assumption here is that there will not be two measurements for the
      // same landmark
      VLOG(2) << fmt::format("No data association for observation idx: {}, creating a new landmark and adding to map", meas_idx);
      lm_id = CreateLandmark(observations.at(meas_idx), state_id);

      // and create the data association (for the factor which will be created later)
      data_association[meas_idx] = lm_id;

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


////////////////////////////////////////////////////////////////
/// \brief Estimator::SetLocalizationMode
////////////////////////////////////////////////////////////////
void Estimator::SetLocalizationMode(bool localization_only) {
  if(localization_mode_ == localization_only) {
    return;
  }

  if (options_.provide_map) {
    VLOG(1) << " Localizatio mode toggled but prior map was provided";
    return;
  }

  VLOG(1) << "Localization mode requested with flag: " << localization_only;
  for (const auto e : landmarks_) {
    if (!e.second->active) { continue; }  // check if landmark is in problem

    if (localization_only) {
      VLOG(1) << " Setting landmark id: " << e.first << " constant." ;
      ceres_problem_->SetParameterBlockConstant(e.second->data());
      e.second->active = false;
    } else {
      ceres_problem_->SetParameterBlockVariable(e.second->data());
      e.second->active = true;
    }
  }
  localization_mode_ = localization_only;
}

////////////////////////////////////////////////////////////////
/// \brief Estimator::CheckStateExists
////////////////////////////////////////////////////////////////
bool Estimator::CheckStateExists(uint64_t state_id) {
  return states_.find(state_id) != states_.end();
}

////////////////////////////////////////////////////////////////////////////
/// \brief Estimator::CheckLandmarkExists
//////////////////////////////////////////////////////////////////////////
bool Estimator::CheckLandmarkExists(uint64_t lm_id) {
  return landmarks_.find(lm_id) != landmarks_.end();
}

//////////////////////////////////////////////////////////////////////////
/// \brief Estimator::LandmarkFromMeasurement
////////////////////////////////////////////////////////////////////////
Landmark Estimator::LandmarkFromMeasurement(uint64_t state_id, RangeFinderObservation obs) {
  if (!CheckStateExists(state_id)) {
    LOG(ERROR) << fmt::format("Error obtaining landmark from state. State id {} requested but not in states map.", state_id);
    return Landmark();
  }

  StatePtr& state = states_.at(state_id);

  // get the landmark in the robot reference frame
  Landmark lm_r(obs.observation.range * std::cos(obs.observation.theta),
                obs.observation.range * std::sin(obs.observation.theta));

  // transfer the landmark over to the world frame
  Landmark lm_w = state->robot * lm_r;

  return lm_w;
}


//////////////////////////////////////////////////////////////////////////
/// \brief Estimator::UpdateLandmarkMean
////////////////////////////////////////////////////////////////////////
void Estimator::UpdateLandmarkMean(uint64_t landmark_id) {
  if (unitialized_landmarks_.find(landmark_id) != unitialized_landmarks_.end()) {

    std::vector<double> x_vals;
    std::vector<double> y_vals;

    std::map<uint64_t, RangeFinderObservationVector> state_2_obs_map = unitialized_landmarks_.at(landmark_id);
    // for every state that observed this landmark
    for (const auto& state  : state_2_obs_map) {
      uint64_t state_id = state.first;

      for (const auto& obs : state.second) {
        // obtain the landmark from this state
        Landmark lm = LandmarkFromMeasurement(state_id, obs);
        x_vals.push_back(lm.x());
        y_vals.push_back(lm.y());
      }
    }
    // get the mean
    double x_avg = std::accumulate(x_vals.begin(), x_vals.end(), 0.0) / x_vals.size();
    double y_avg = std::accumulate(y_vals.begin(), y_vals.end(), 0.0) / y_vals.size();

    // and update the landmark
    landmarks_.at(landmark_id)->SetPosition(x_avg, y_avg);
  }
}


////////////////////////////////////////////////////////////////////////
/// \brief Estimator::CheckAndAddObservationFactors
////////////////////////////////////////////////////////////////////////
void Estimator::CheckAndAddObservationFactors(const uint64_t state_id,
                                              const DataAssociationMap& data_asssociation, const RangeFinderObservationVector &observations) {

  for (const auto& e : data_asssociation) {
    // association is: index in observation vector -> landmark_id

    uint64_t landmark_id = e.second;
    size_t meas_idx = e.first;

    LandmarkPtr lm = landmarks_.at(landmark_id);
    lm->AddObservation();

    if (lm->GetNumObservations() < options_.delayed_initalization_num && !options_.provide_map) {
      VLOG(2) << fmt::format("landmark id: {} has {} observations, but {} are needed.", landmark_id, lm->GetNumObservations(),
                             options_.delayed_initalization_num);
      unitialized_landmarks_[landmark_id][state_id].push_back(observations.at(meas_idx)); // save observation for later.
    }
    else if (unitialized_landmarks_.find(landmark_id) != unitialized_landmarks_.end() &&
             lm->GetNumObservations() == options_.delayed_initalization_num) {

      VLOG(2) << fmt::format("have enough measurements to initalize landmark id: {}", landmark_id);

      unitialized_landmarks_[landmark_id][state_id].push_back(observations.at(meas_idx)); // save the current observation.
      UpdateLandmarkMean(landmark_id);

      // add a factor for every observation we saved
      for (const auto& states  : unitialized_landmarks_.at(landmark_id)) {
        // for every observation from that state
        for (const auto& obs : states.second) {
          // obtain the landmark from this state
          uint64_t s_id = states.first;
          CreateObservationFactor(s_id, landmark_id, obs);
        }
      }

      // remove the landmark from the uninitialized list
      unitialized_landmarks_.erase(unitialized_landmarks_.find(landmark_id));
    }
    else {
      CreateObservationFactor(state_id, landmark_id, observations.at(meas_idx));
    }
  }
}


////////////////////////////////////////////////////////////////////////
/// \brief Estimator::CreateObservationFactor
////////////////////////////////////////////////////////////////////////
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

  // make sure lm is marked as active since it's part of the estimation now
  if (!localization_mode_) {
    landmarks_.at(landmark_id)->active = true;
  }

  // and that it has a persistence filter associated
  if (options_.filter_options.use_persistence_filter) {
    if (persistence_filter_map_.find(landmark_id) == persistence_filter_map_.end()) {
      // create a new persistence filter for this landmark
      persistence_filter_map_.insert({landmark_id, std::make_shared<PersistenceFilter>(logS_T)});
    }

    // and add this observation
    double P_M = options_.filter_options.P_M;
    if (persistence_filter_graph_.find(landmark_id) != persistence_filter_graph_.end()){
      for (const auto& e : persistence_filter_graph_[landmark_id]) {
        if (persistence_filter_map_.find(e.first) != persistence_filter_map_.end()) {
          P_M *= (e.second - (e.second-1) * landmarks_.at(e.first)->persistence_prob);
        }
      }
    }

    if ((landmark_id == 27 || landmark_id == 28 || landmark_id == 1 || landmark_id == 2) &&
        persistence_filter_map_.find(26) != persistence_filter_map_.end() &&
        landmarks_.at(26)->persistence_prob < 0.3){
      P_M = 0.8;

    }
    persistence_filter_map_.at(landmark_id)->update(true, obs.time+1, P_M,
                                                    options_.filter_options.P_F);

  }

  // add observation factor between state and landmark to problem
  RangeFinderCovariance range_cov = RangeFinderReading::GetMeasurementCovariance();
  if (options_.weigh_landmarks_by_persistence && options_.filter_options.use_persistence_filter &&
      persistence_filter_map_.find(landmark_id) != persistence_filter_map_.end()) {
    double persistence_prob = landmarks_.at(landmark_id)->persistence_prob;
    if(persistence_prob < 1e-10){
      persistence_prob = 1e-10;
    }
    range_cov *= 1.0 / persistence_prob;
  }
  Eigen::LLT<RangeFinderCovariance> llt_of_information(range_cov.inverse());
  RangeFinderCovariance sqrt_information = llt_of_information.matrixL().transpose();

  auto observation_cost_function = new ::ceres::AutoDiffCostFunction<ceres::RangeFinderObservationCostFunction,
      RangeFinderReading::kMeasurementDim, Sophus::SE2d::num_parameters, Landmark::kLandmarkDim>(
        new ceres::RangeFinderObservationCostFunction (obs, sqrt_information));

  ceres_problem_->AddResidualBlock(observation_cost_function, ceres_loss_function_.get(),
                                   states_.at(state_id)->data(), landmarks_.at(landmark_id)->data());
  VLOG(2) << fmt::format("Created observation factor between state {} and landmark {}", state_id, landmark_id);

}


//////////////////////////////////////////////////////////////////////////////
/// \brief Estimator::CreateOdometryFactor
///////////////////////////////////////////////////////////////////////////
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
  odometry_cov(0,0) = Square(5e-2);
  odometry_cov(1,1) = Square(1e-3);
  odometry_cov(2,2) = Square(5e-2);
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


//////////////////////////////////////////////////////////////////////////////
/// \brief Estimator::CreateLandmark
/////////////////////////////////////////////////////////////////////////
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
  lm_w.active = false;
  landmarks_.insert({lm_id, std::make_shared<Landmark>(lm_w)});
  VLOG(2) << fmt::format("Created landmark observed from state id: {}, lm id: {}", state_id, lm_id);
}


//////////////////////////////////////////////////////////////////////////////
/// \brief Estimator::CreateLandmark
/////////////////////////////////////////////////////////////////////////
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
  lm_w.active = false;

  uint64_t lm_id = IdGenerator::Instance::NewId();
  landmarks_.insert({lm_id, std::make_shared<Landmark>(lm_w)});

  VLOG(2) << fmt::format("Created landmark observed from state id: {}, lm id: {}", state_id, lm_id);
  return lm_id;
}

///////////////////////////////////////////////////////////////////
/// \brief Estimator::GetFullJacobian
/////////////////////////////////////////////////////////////////////
bool Estimator::GetFullJacobian() {
  VLOG(1) << "Getting problem jacobian.";
  // we want the parameter blocks ordered (by increasing id) poses, then landmarks
  std::vector<double*> parameter_blocks;
  std::vector<::ceres::ResidualBlockId> residual_blocks; // just use the order that the residual blocks were added to the problem for now

  // get all the robot poses
  for (const auto& s : states_) {
    if (!s.second->fixed && s.second->active) {
      parameter_blocks.push_back(s.second->data());
    }
  }

  // and now the landmarks
  for (const auto& l : landmarks_) {
    if (l.second->active) {
      parameter_blocks.push_back(l.second->data());
    }
  }

  double cost = 0;
  std::vector<double> residuals;
  std::vector<double> gradient;
  ::ceres::CRSMatrix sparse_jacobian;
  ::ceres::Problem::EvaluateOptions options;
  options.apply_loss_function = false;
  options.parameter_blocks = parameter_blocks;
  options.residual_blocks = residual_blocks;

  if (!ceres_problem_->Evaluate(options, &cost, &residuals, &gradient, &sparse_jacobian)) {
    LOG(ERROR) << "Error calculating jacobian, problem evaluation failed.";
    return false;
  }

  Eigen::MatrixXd jacobian(sparse_jacobian.num_rows, sparse_jacobian.num_cols);
  jacobian.setZero();

  for (size_t i = 0; i < sparse_jacobian.rows.size()-1; i++) {
    size_t start = sparse_jacobian.rows[i];
    size_t end = sparse_jacobian.rows[i+1] - 1;
    for(size_t j = start; j <= end; j++){
      jacobian(i, sparse_jacobian.cols[j]) = sparse_jacobian.values[j];
    }
  }

  VLOG(1) << "printing jacobian to file";
  std::ofstream("j.txt", std::ios_base::trunc) << jacobian.format(kLongCsvFmt);


  return true;

}

///////////////////////////////////////////////////////////////////
/// \brief Estimator::GetEstimationResult
/////////////////////////////////////////////////////////////////////
void Estimator::GetEstimationResult(EstimatedData* data)  {
  if(data != nullptr) {
    if(options_.filter_options.use_persistence_filter) {
      UpdateMapPersistence();
    }
    data->landmarks = landmarks_;
    data->states = states_;
    data->data_association = data_assoc_results_;
  }
}


///////////////////////////////////////////////////////////////////
/// \brief Estimator::UpdateMapPersistence
/////////////////////////////////////////////////////////////////////
void Estimator::UpdateMapPersistence() {
  for (auto& e : landmarks_) {
    if (persistence_filter_map_.find(e.first) != persistence_filter_map_.end()) {
      e.second->persistence_prob = persistence_filter_map_.at(e.first)->predict(latest_timestamp_ + 1);
    }
  }
}

}  // namespace ceres
}  // namespace chameleon
