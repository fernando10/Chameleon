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
  std::map<uint64_t, double> min_lm_cost;
  Eigen::VectorXd landmark_ids(map.size());

  const size_t num_landmarks = map.size();

  Eigen::MatrixXd cost_matrix(measurements.size(), measurements.size() + num_landmarks);

  // set the threshold cost for spurious measurements
  for (size_t r = 0; r < measurements.size(); ++r) {
    for (size_t col = 0; col < measurements.size(); ++col) {
      cost_matrix(r, num_landmarks + col) = D_max;
    }
  }

  size_t meas_idx = 0;
  for (const RangeFinderObservation& z : measurements) {
    double min_cost = D_max*2; // uninitialized, above max threshold

    VLOG(2) << " Pairing measurement idx: "<< meas_idx << " the correct pairing is with lm: "  << z.observation.lm_id;
    size_t map_idx = 0;
    for (const auto& e : map) {
      VLOG(2) << " Testing lm id: "  << e.first << "("  << z.observation.lm_id << ")";
      const LandmarkPtr& lm = e.second;

      Eigen::MatrixXd H;
      RangeFinderReading prediction = current_state.robot.Predict(*lm, &H);
      //current_state.robot.CheckPredictionJacobian(*lm);

      VLOG(2) << fmt::format(" got prediction: {} " , prediction.vec().transpose());
      VLOG(2) << fmt::format(" measurement: {} " , z.observation.vec().transpose());

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

      RangeFinderCovariance C = H * sigma * H.transpose() + RangeFinderReading::GetMeasurementCovariance();

      double dist = DataAssociation::MahalanobisDistance(z.observation.vec(), Distribution(prediction.vec(), C));
      VLOG(2) << " cost: "  << dist;

      //      if (dist < D_max && dist < min_cost) {
      //        min_cost = dist;
      //        // check if this landmark isn't already associated to another measurement with a lower cost
      //        if (min_lm_cost.find(e.first) == min_lm_cost.end() || min_lm_cost.at(e.first) > dist ) {


      //          if (min_lm_cost.find(e.first) != min_lm_cost.end()) {
      //            // another observation was already associated to this landmark, remove that associatoin since we can only have one measurement
      //            // per landmark
      //            for (DataAssociationMap::iterator it = association->begin(); it != association->end(); ++it) {
      //              if (it->second == e.first) {
      //                VLOG(1) << fmt::format(" Removing association from obs: {} to lm {}", it->first, e.first);
      //                association->erase(it);
      //                break;
      //              }
      //            }
      //          }
      //          min_lm_cost[e.first] = dist;
      //          (*association)[meas_idx] = e.first;
      //          VLOG(1) << fmt::format("Associating meas {} to lm {}, cost: {}", meas_idx, e.first, min_cost);
      //        } else {
      //          VLOG(1) << fmt::format("landmark id {} is already assigned to a meas, with cost: {} which is lower than the cost {} for assigning to meas {}",
      //                                 e.first, min_lm_cost.at(e.first), dist, meas_idx);
      //        }
      //      }
      // store the cost
      cost_matrix(meas_idx, map_idx) = dist;
      landmark_ids[map_idx] = e.first;
      map_idx++;
    }
    meas_idx++;
  }

  for (int r = 0; r < cost_matrix.rows(); ++r) {
    double min_cost = 100;
    for (int c = 0; c < cost_matrix.cols(); ++ c) {
      if(cost_matrix(r, c) < min_cost) {
        min_cost = cost_matrix(r, c);
        if (min_cost < D_max) {
          VLOG(2) << " Associating obs idx " << r << " to lm id: " << landmark_ids[c];
          (*association)[r] = landmark_ids[c];
        }
      }
    }
  }

  //  // go through the columns assigned to each row (measurement) in the cost matrix
  //  for (size_t i = 0; i < row_solution.size(); ++i) {
  //    VLOG(2) << "assigning meas idx: "  << i << " to lm id"  << landmark_ids[row_solution[i]];
  //    (*association)[i] = landmark_ids[row_solution[i]];
  //  }

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
