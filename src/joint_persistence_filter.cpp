#include "chameleon/joint_persistence_filter.h"
#include "chameleon/persistence_filter_utils.h"
#include "glog/logging.h"

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
  if (detections.size() > num_features_) {
    LOG(FATAL) << "Trying to update joint persistence filter with " << detections.size() << " detections but we are only expecting up to " << num_features_;
  }

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
  }




}

double JointPersistenceFilter::PredictMarginal(double prediction_time, uint64_t feature_id) const {
  return 0;
}


}  // namespace chameleon
