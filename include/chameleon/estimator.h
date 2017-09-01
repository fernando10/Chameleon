// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include "ceres/ceres.h"
#include "ceres/problem.h"
#include <Eigen/Core>
#include <unordered_map>
#include "chameleon/types.h"
#include "chameleon/util.h"
#include "chameleon/data_association.h"
#include "chameleon/auto_diff_local_param_SE2.h"
#include "chameleon/persistence_filter.h"

namespace chameleon
{
namespace ceres
{

class Estimator {
public:

  struct PersistenceFilterOptions {
    double P_M = 0.2;
    double P_F = 0.2;
    bool use_persistence_filter = false;
    bool use_joint_persistence = false;
  };

  struct EstimatorOptions {
    DataAssociation::DataAssociationType data_association_strategy = DataAssociation::DataAssociationType::Known;
    ::ceres::Solver::Options ceres_options;
    bool print_full_summary = false;
    double huber_loss_a = 1.0;
    bool print_brief_summary = true;
    PersistenceFilterOptions filter_options;
    bool add_observations = true;
    bool provide_map = true;
    bool weigh_landmarks_by_persistence = false;
    bool compute_landmark_covariance = true;
    bool compute_latest_pose_covariance = true;
    size_t delayed_initalization_num = 10;  // wait till we have this many observations to initialize a landmark //TODO: add landmark at first observation but with high uncertainty
    size_t min_states_for_solve = 3;  // number of states to have in optimization window before we call solve
  };

  Estimator(const EstimatorOptions& options);

  ///
  /// \brief AddData
  /// - Create and initialize a new state
  /// - Associate measurements to exisiting landmarks
  /// - Create new landmarks if necessary
  /// - Add node to graph with 1 odometry and N measurement factors
  /// \param data RobotData containing odometry measurement and observations for the current timestep
  ///
  void AddData(const RobotData& data);

  ///
  /// \brief SetMap
  /// - Provide a pre-built map that can be used for localization
  /// \param prior_map
  ///
  void SetMap(LandmarkVectorPtr prior_map);

  ///
  /// \brief Solve
  /// - Solve the full problem (all poses + landmarks)
  void Solve();

  void Reset();
  void SetLocalizationMode(bool localization_only);
  void GetEstimationResult(EstimatedData* data);
  bool GetFullJacobian();
  void SetPersistenceWeights(FeaturePersistenceWeightsMapPtr weights) {
    persistence_weights_ = weights;
  }


private:


  ///
  /// \brief Estimator::CreateNewLandmarks
  /// - Check if any observations do not have a data association
  /// - Crate a new landmark at the observed locatoin of that observation
  /// - Associate landmark with the state in which it was observed
  /// - Add observation factor to problem
  /// \param data_association > association from measurements to existing landmarks
  /// \param observations > list of observations
  void CreateNewLandmarks(DataAssociationMap& data_association, const RangeFinderObservationVector& observations,
                          uint64_t state_id);

  ///
  /// \brief Estimator::CreateLandmark
  /// \param obs observation (range and bearing)
  /// \param state_id (the state from which this landmark was observed)
  /// \return
  uint64_t CreateLandmark(const RangeFinderObservation& obs, uint64_t state_id);
  void CreateLandmark(const RangeFinderObservation &obs, uint64_t state_id, uint64_t lm_id);
  void UpdateLandmarkMean(uint64_t landmark_id);
  uint64_t GetNewStateId();
  uint64_t GetNewLandmarkId();
  void CreateObservationFactor(const uint64_t state_id, const uint64_t landmark_id, const RangeFinderObservation& obs);
  void CreateOdometryFactor(const uint64_t prev_state_id, const uint64_t current_state_id, const RobotData& data);
  bool CheckStateExists(uint64_t state_id);
  bool CheckLandmarkExists(uint64_t lm_id);
  Landmark LandmarkFromMeasurement(uint64_t state_id, RangeFinderObservation obs);
  void CheckAndAddObservationFactors(const uint64_t state_id, const DataAssociationMap& data_asssociation, const RangeFinderObservationVector& observations);
  void GetMapUncertainty();
  bool GetLandmarkUncertainty(std::vector<uint64_t> landmark_ids, Eigen::MatrixXd* cov_out);
  bool GetLatestPoseUncertainty();
  bool GetMarginals(uint64_t state_id, std::vector<uint64_t> lm_ids, Marginals* res);
  bool GetMarginals(uint64_t state_id, LandmarkPtrMap lms, Marginals* res);

  ///
  /// \brief UpdateMapPersistence
  ///Predict the persistence posterior for all the landmakrs in the map
  void UpdateMapPersistence();
  LandmarkPtrMap GetLandmarksThatShouldBeVisible(const RobotPose& robot, bool use_robot_fov = false);

  std::unique_ptr<::ceres::Problem> ceres_problem_;
  std::unique_ptr<::ceres::LossFunction> ceres_loss_function_;
  std::unique_ptr<::ceres::LocalParameterization> local_param_;
  ::ceres::Solver::Summary summary_;

  std::unordered_map<uint64_t, std::map<uint64_t, double>> persistence_filter_graph_;
  std::unordered_map<uint64_t, PersistenceFilterPtr> persistence_filter_map_;
  std::unordered_map<uint64_t, std::map<uint64_t, RangeFinderObservationVector>> unitialized_landmarks_;
  LandmarkPtrMap landmarks_;
  StatePtrMap states_;
  const EstimatorOptions& options_;
  bool localization_mode_ = false;
  double latest_timestamp_;
  uint64_t last_state_id_;
  DataAssociationResults data_assoc_results_;
  FeaturePersistenceWeightsMapPtr persistence_weights_;
  State2Landmark_Multimap state_2_landmark_multimap_;
  Landmark2State_MultiMap landmark_2_state_multimap_;
};

}  // namespace ceres
}  // namespace chameleon
