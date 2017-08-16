#include "chameleon/data_association.h"
#include "glog/logging.h"

namespace chameleon
{

DataAssociationMap DataAssociation::AssociateDataToLandmarks(const RangeFinderObservationVector& measurements,
                                                             const LandmarkPtrMap& map, const StatePtr& current_state, DataAssociationType strategy) {
  DataAssociationMap association;
  switch (strategy) {
  case DataAssociationType::NN:
    LOG(ERROR) << " NN Not implemented";
    break;
  case DataAssociationType::IC:
    LOG(ERROR) << " IC Not implemented";
    break;
  case DataAssociationType::JCBB:
    LOG(ERROR) << " JCBB Not implemented";
    break;
  case DataAssociationType::Known:
    KnownDataAssociation(measurements, map, &association);
    break;
  }
  return association;
}

double DataAssociation::MahalanobisDistance(Eigen::VectorXd z, Distribution dist) {
  Eigen::VectorXd res = z - dist.mean;
  return res.transpose() * dist.cov.inverse() * res;  // could be costly for large C
}

void DataAssociation::IndividualCompatibility(const RangeFinderObservationVector& measurements, const LandmarkPtrMap& map,
                                              const StatePtr& current_state, DataAssociationMap* const  association){
  // we compute the individual compatibility by thresholding the mahalanobis distance of the innovation:
  //
  // let x be our current state (pose and landmarks): x ~ N(x_bar, \Sigma)
  // and z be our candidate measurement: z ~ N(h(x_bar), Q)
  // the log likelihood of z is then: || z - h(x_bar)||_Q
  // whre Q = H * Sigma * H^T + Q
  // and H is the derivative of h(x) w.r.t. x

  size_t meas_idx = 0;
  for (const RangeFinderObservation& z : measurements) {
    double max_cost = -1; // uninitialized, mahalanobis distance is always >= 0 so no issues
    for (const auto& e : map) {
      LandmarkPtr& lm = e.second;
      // predict the measurement
      Eigen::Vector2d lm_r = util::DeHomogenizeLandmark<double>(current_state->robot.pose.inverse().matrix() *
                                                                util::HomogenizeLandmark<double>(lm->vec()));
      double range_pred = lm_r.norm();
      double bearing_pred = AngleWraparound<double>(std::atan2(lm_r.y(), lm_r.x()));
      RangeFinderReading prediction(bearing_pred, range_pred);

      // build the state covariance matrix (robot pose + 1 landmark)
      Eigen::Matrix<double, Landmark::kLandmarkDim + State::kStateDim, Landmark::kLandmarkDim + State::kStateDim> sigma;
      sigma.setZero();
      // here we need the state-state, lm-lm and state-lm covariance blocks
      // Sigma = [ State-State State-LM
      //                    LM-State       LM-LM]

      // now build the covariance matrix for the test
      Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();

    }
    meas_idx++;
  }





}

void DataAssociation::KnownDataAssociation(const RangeFinderObservationVector& measurements, const LandmarkPtrMap& /*map*/
                                           , DataAssociationMap* const  association) {
  VLOG(1) << "Getting KNOWN data associations";

  size_t meas_idx = 0;
  for (const RangeFinderObservation& obs : measurements) {
    association->insert({meas_idx, obs.observation.lm_id});
    meas_idx++;
  }
  VLOG(1) << fmt::format("Got {} data associations for {} measurements" , association->size(), measurements.size());
}

}  // namespace chameleon
