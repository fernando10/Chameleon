#include "chameleon/joint_persistence_filter.h"
#include "chameleon/persistence_filter_utils.h"
#include "glog/logging.h"
#include <gsl/gsl_sf_log.h>
#include <gsl/gsl_sf_exp.h>
#include <gsl/gsl_errno.h>

namespace chameleon
{

double JointPersistenceFilter::Predict(double prediction_time) const {

  // Input checking
  if(prediction_time < latest_obs_time_)
  {
    LOG(FATAL) << "Prediction time must be at least as recent as the last incorporated observation (prediction_time >= last_observation_time)";
  }

  return 0;
}

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
    log_observation_likelihood_ += log_p_j_tNplus1; // also update the total running sum for the joint log likelihood
  }

  // update the latest observation time
  latest_obs_time_ = observation_time;




}

double JointPersistenceFilter::PredictMarginal(double prediction_time, uint64_t feature_id) const {
  return 0;
}


}  // namespace chameleon
