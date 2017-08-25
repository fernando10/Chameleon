// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
// The joint persistence filter carries the joint posterior:
// P(Theta_{1:M} = 1 | J_{1:N})
// where Theta = {Theta_1, Theta_2..., Theta_M}, the joint persistence of all M features
// and J_{1_N} = {J_{1_{1:N}}, J_{2_{1:N}}...J_{M_{1:N}}} the sequences of detections for
// each M feature from timestep 1 through N
//
// Parts of this file are inspired by the repository https://github.com/david-m-rosen/persistence_filter
// which relates to the paper"Towards Lifeling Feature-Based Mapping in Semi-Static Environments"
// Rosen et al, ICRA 2016
#pragma once
#include <cstddef>
#include <gsl/gsl_sf_exp.h>
#include <functional>
#include <memory>
#include "glog/logging.h"
#include "fmt/format.h"
#include "fmt/ostream.h"
#include <Eigen/Core>

namespace chameleon
{
class JointPersistenceFilter
{
public:
  JointPersistenceFilter(const std::function<double(double)>& log_survival_function, double init_time = 0.0,
                         std::vector<uint64_t> feature_ids) :
    init_time_(init_time), logS_(log_survival_function), feature_ids_(feature_ids),
    latest_obs_time_(init_time), log_evidence_sum_(nullptr), log_observation_likelihood_(0.0), log_marginal_(0.0){

    // simple lambda to shift the time (used to adjust the current time by the start time)
    auto time_shifter = [](double time, double time_shift){ return time - time_shift; };
    shifted_logS_ = std::bind(logS_, std::bind(time_shifter, std::placeholders::_1, init_time_));

    // initialize each individual feature's running observation likelihood
    for (const auto& id : feature_ids_) {
      feature_log_likelihoods_[id] = 0.0;
    }
  }

  // Adds new observations to the filter, size of observation map needs to be less than or equal to the number of features being tracked
  void Update(std::map<uint64_t, bool> detections, double observation_time, double P_M, double P_F);

  // computes the joint posterior at any time t greater than the lastet observation
  double Predict(double prediction_time) const;

  // computes the marginal posterior for the given feature at any time t greater than the lastest time
  double PredictMarginal(double prediction_time, uint64_t feature_id) const;

  double LastObservationTime() const {
    return latest_obs_time_;
  }




private:
  double init_time_; // time filter was initialized...prior probability of existence is 1 here
  std::function<double(double)> logS_;
  std::function<double(double)> shifted_logS_;
  std::vector<uint64_t> feature_ids_;
  double latest_obs_time_;
  std::vector<uint64_t> feature_ids_;  // id's of the features that we are jointly tracking the persistence of
  std::map<uint64_t, std::vector<bool>> feature_observations_;  // keeps track of the sequence of observations for each feature
  std::map<uint64_t, double> feature_log_likelihoods_;  // log likelihoods for each feature's measurements given t = t_n (latest time)
  Eigen::MatrixXd persistence_weights_;  // matrix for the weights that correlate the persistence of individual features

  // The natural logarithm of the joint likelihood probability p(J_{1:M} | t_N) for all features given the current time
  double log_observation_likelihood_;

  // The natural logarithm of the lower evidence sum L(J_{1:M})
  double* log_evidence_sum_;

  // The natural logarithm of the marginal (evidence) probability p(J_{1:M})
  double log_marginal_;



};

typedef std::shared_ptr<JointPersistenceFilter> JointPersistenceFilterPtr;


}  // namespace chameleon
