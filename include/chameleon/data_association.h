// Copyright 2017 Toyota Research Institute.  All rights reserved.
// Data Association class, tasked with:
// - Given a set of measurements and the existing map
//     return the measurement <-> landmark association, if it exists
//     there can only be 1 measurement per landmark, unassociated
//     measurements may be a new landmark or spurious measurement
#pragma once

#include "chameleon/types.h"

namespace chameleon
{

class DataAssociation {
public:
  DataAssociation(){}

  enum class DataAssociationType {
    NN,          // Nearest Neighbor
    IC,           // Individual Compatibility -> Mahalanobis Distance
    JCBB,      // Joint Compatability Branch and Bound
    Known   // Simulated data or external input
  };

  ///
  /// \brief AssociateDataToLandmarks
  /// \param measurements vector of range and bearing measurements
  /// \param map exsiting map of landmarks
  /// \param strategy what data association strategy to use
  /// \return Data association in the form: index of observation in measurement vector -> landmark_id
  ///
  static DataAssociationMap AssociateDataToLandmarks(const RangeFinderObservationVector& measurements,
                                                     const LandmarkPtrMap& map, const StatePtr& current_state, DataAssociationType strategy);

private:

  // Get the data association directly from the measurements, the map isnt needed for this one
  static void KnownDataAssociation(const RangeFinderObservationVector& measurements, const LandmarkPtrMap& map,
                                   DataAssociationMap* const  association);

  // Don't assume known data association and use the mahalanobis distance to establish the best pairing for each landmark
  void IndividualCompatibility(const RangeFinderObservationVector& measurements, const LandmarkPtrMap& map
                                                , const StatePtr& current_state, DataAssociationMap* const  association);

  double MahalanobisDistance(Eigen::VectorXd z, Distribution dist);
};

}  //namespace chameleon
