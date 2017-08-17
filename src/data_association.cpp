#include "chameleon/data_association.h"
#include "glog/logging.h"
namespace chameleon
{

DataAssociationMap DataAssociation::AssociateDataToLandmarks(const RangeFinderObservationVector& measurements,
                                                             const LandmarkPtrMap& map, const Marginals& current_state,
                                                             DataAssociationType strategy) {
  VLOG(1) << fmt::format("Getting data associations for {} measurements. {} landmarks passed in as candidates.",
                         measurements.size(), map.size());
  DataAssociationMap association;
  switch (strategy) {
  case DataAssociationType::NN:
    LOG(ERROR) << " NN Not implemented";
    break;
  case DataAssociationType::IC:
    IndividualCompatibility(measurements, map, current_state, &association);
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
                                              const Marginals& current_state, DataAssociationMap* const  association){
  //  // we compute the individual compatibility by thresholding the mahalanobis distance of the innovation:
  //  //
  //  // let x be our current state (pose and landmarks): x ~ N(x_bar, \Sigma)
  //  // and z be our candidate measurement: z ~ N(h(x_bar), Q)
  //  // the log likelihood of z is then: || z - h(x_bar)||_Q
  //  // whre Q = H * Sigma * H^T + Q
  //  // and H is the derivative of h(x) w.r.t. x

  const double D_max = 9.210;  // 2 dof chi^2 thresh.

  size_t meas_idx = 0;
  for (const RangeFinderObservation& z : measurements) {
    double min_cost = D_max*2; // uninitialized, above max threshold

    for (const auto& e : map) {
      const LandmarkPtr& lm = e.second;
      // predict the measurement
      Eigen::Vector2d lm_r = util::DeHomogenizeLandmark<double>(current_state.robot.pose.inverse().matrix() *
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
      if (current_state.covariances.empty()) {
        sigma.setIdentity();
      } else {
        sigma.topLeftCorner<State::kStateDim, State::kStateDim>() = current_state.covariances.at(std::make_pair(current_state.pose_id,
                                                                                                                current_state.pose_id));
        sigma.block<Landmark::kLandmarkDim, Landmark::kLandmarkDim>(State::kStateDim, State::kStateDim) =
            current_state.covariances.at(std::make_pair(e.first, e.first));

        sigma.block<State::kStateDim, Landmark::kLandmarkDim>(0, State::kStateDim) =
            current_state.covariances.at(
              std::make_pair(current_state.pose_id, e.first));
        sigma.block<Landmark::kLandmarkDim, State::kStateDim>(State::kStateDim, 0) =
            current_state.covariances.at(
              std::make_pair(current_state.pose_id, e.first)).transpose();
      }


      // now build the covariance matrix for the test
      Eigen::Matrix<double, 2, State::kStateDim + Landmark::kLandmarkDim> H =
          Eigen::Matrix<double, 2, State::kStateDim + Landmark::kLandmarkDim>::Zero();
      H.block<2, State::kStateDim>(0, 0) = -Eigen::Matrix<double, 2, State::kStateDim>::Identity();
      H.block<2, Landmark::kLandmarkDim>(0, State::kStateDim) = Eigen::Matrix<double, 2, Landmark::kLandmarkDim>::Identity();

      RangeFinderCovariance C = H * sigma * H.transpose() + RangeFinderReading::GetMeasurementCovariance();

      double dist = DataAssociation::MahalanobisDistance(z.observation.vec(), Distribution(prediction.vec(), C));

      if (dist < D_max && dist < min_cost) {
        min_cost = dist;
        (*association)[meas_idx] = e.first;
        VLOG(1) << fmt::format("Associating meas {} to lm {}, cost: {}", meas_idx, e.first, min_cost);
      }
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
