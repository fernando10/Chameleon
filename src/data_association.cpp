#include "chameleon/data_association.h"
#include "glog/logging.h"

namespace chameleon
{

DataAssociationMap DataAssociation::AssociateDataToLandmarks(const RangeFinderObservationVector& measurements,
                                                             const LandmarkPtrMap& map, DataAssociationType strategy) {
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
