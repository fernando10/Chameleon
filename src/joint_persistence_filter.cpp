#include "chameleon/joint_persistence_filter.h"
#include "chameleon/persistence_filter_utils.h"
#include "glog/logging.h"
#include <gsl/gsl_sf_log.h>
#include <gsl/gsl_sf_exp.h>
#include <gsl/gsl_errno.h>

namespace chameleon
{

//double JointPersistenceFilter::Predict(double prediction_time, uint64_t lm_id) const {

//  // Input checking
//  if(prediction_time < latest_obs_time_)
//  {
//    LOG(FATAL) << "Prediction time must be at least as recent as the last incorporated observation (prediction_time >= last_observation_time)";
//  }

//  return 0;
//}

void JointPersistenceFilter::Update(std::map<uint64_t, bool> detections, double observation_time,
                                    double P_M, double P_F) {
  if(observation_time < latest_obs_time_)
  {
    std::string error = "Current observation must be at least as recent as the last incorporated observation (observation_time >= last_observation_time)";
    LOG(FATAL) << error;
  }

  if( (P_M < 0) || (P_M > 1) )
  {
    std::string error = "Probability of missed detection must be between 0 and 1";
    LOG(FATAL) << error;
  }

  if( (P_F < 0) || (P_F > 1) )
  {
    std::string error = "Probability of false alarm must be between 0 and 1";
    LOG(FATAL) << error;
  }

  // we have to update the joint evidence and the likelihood terms

  // first update the likelihood terms as these factor independently
  // P(J_{1:M} | Theta_{1:M}) = Prod_{i=1:M} P(J_{i} | Theta_{i})
  for (const auto& e : detections) {
    uint64_t feature_id = e.first;
    bool detector_output = e.second;

    if (feature_log_likelihoods_.find(feature_id) == feature_log_likelihoods_.end()) {
      LOG(ERROR) << fmt::format("feature id {} provided but that id is not a part of this filter", feature_id);
      continue;
    }
    double log_p_j_tNplus1 = detector_output ? gsl_sf_log(1.0 - P_M) : gsl_sf_log(P_M);
    feature_log_likelihoods_.at(feature_id) += log_p_j_tNplus1;
    //log_observation_likelihood_ += log_p_j_tNplus1; // also update the total running sum for the joint log likelihood

    // also update the evidence
//    if (log_evidence_sum_ != nullptr) {
//      *log_evidence_sum_ += logsum(*log_evidence_sum_, log_observation_likelihood_ + shifted_logdF(observation_time, latest_obs_time_)) + (detector_output ? gsl_sf_log(P_F) : gsl_sf_log(1 - P_F));
//    } else {

//      double log1;
//      gsl_error_handler_t* error_handler = gsl_set_error_handler_off();
//      gsl_sf_result result;
//      int status = gsl_sf_exp_e(shifted_logS_(observation_time), &result);
//      gsl_set_error_handler(error_handler);

//      if(status == GSL_SUCCESS)
//      {
//        log1 = gsl_sf_log_1plusx(-result.val);
//      }
//      else
//      {
//        log1 = 0.0;
//      }

//      log_evidence_sum_ = new double(((detector_output ? gsl_sf_log(P_F) : gsl_sf_log(1 - P_F)) + log1_minus_ST));

//    }
  }

  // update the latest observation time
  latest_obs_time_ = observation_time;
}

double JointPersistenceFilter::ConditionalPersistencePrior(size_t current_idx, size_t condition_start_idx) {
return 0;
}

double JointPersistenceFilter::Predict(double prediction_time, uint64_t lm_id) const {
  VLOG(2) << fmt::format("Predict joint posterior with  time: {}", prediction_time);

  if(prediction_time < latest_obs_time_)
  {
    LOG(FATAL) << "Prediction time must be at least as recent as the last incorporated observation (prediction_time >= last_observation_time)";
  }

  // first compute the prior persistence probability P(Theta = 1) = P(Theta_1 = 1) * ... * P(Theta_M = 1 | Theta_{M-1} = 1, ..., Theta_1 = 1)
  double log_theta = 0.0;
  for (size_t i = 0; i < feature_ids_.size(); ++i) {
    log_theta += shifted_logS_(prediction_time);
  }

  // compute the full joint posterior as P(Theta = 1 | J_{1:N}) = (P(J_{1:N} | Theta = 1) * P(Theta = 1)) / P(J_{1:N})
  double exp_arg = log_observation_likelihood_ - log_marginal_ + shifted_logS_(prediction_time);
  gsl_error_handler_t* error_handler = gsl_set_error_handler_off();
  gsl_sf_result result;
  int status = gsl_sf_exp_e(exp_arg, &result);
  gsl_set_error_handler(error_handler);

  if(status == GSL_SUCCESS)
  {
    VLOG(3) << "Got prediction: " << result.val;
    return result.val;
  }
  else
  {
    VLOG(3) << "Got prediction: " << 0.0;
    return 0.0;
  }

}

double JointPersistenceFilter::PredictMarginal(double prediction_time, uint64_t feature_id) const {
  return 0;
}


}  // namespace chameleon
