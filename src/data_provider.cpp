// Copyright 2017 Toyota Research Institute.  All rights reserved.
//

#include "chameleon/data_provider.h"
#include "chameleon/util.h"
#include "chameleon/odometry_generator.h"
namespace chameleon
{

DataProvider::DataProvider(std::string data_type, std::string data_file) {
  SetDataType(data_type);
  data_file_ = data_file;
  data_generator_ = util::make_unique<DataGenerator>(sim_options_);
  current_index_ = 0;
  LoadData();
}

void DataProvider::Reset() {
  data_generator_->Reset();
  current_index_ = 0;
}

LandmarkVectorPtr DataProvider::GetPriorMap() {
  LandmarkVectorPtr map;

  switch (type_) {
  case DataType::Sim:
  {
    map = data_generator_->GetNoisyMap();
  }
    break;
  case DataType::UTIAS:
  {
    map = GetUTIASMap();
  }
    break;
  case DataType::VicPark:
  {
    //TODO
  }
    break;
  }

  return map;
}


FeaturePersistenceWeightsMapPtr DataProvider::BuildFeaturePersistenceAssociationMatrix(double self_weight, double num_neighbors, double radius,
                                                                                    double delta_time) {
  return BuildFeaturePersistenceAssociationMatrix(GetPriorMap(), self_weight, num_neighbors, radius, delta_time);
}

FeaturePersistenceWeightsMapPtr DataProvider::BuildFeaturePersistenceAssociationMatrix(const LandmarkVectorPtr& map,
                                                                                       double self_weight,
                                                                                    double num_neighbors, double radius,
                                                                                    double /*delta_time*/) {
  VLOG(1) << "Building persistence weights...";
  // TODO: Switch to sparse matrix
  FeaturePersistenceWeightsMapPtr weights_map = std::make_shared<FeaturePersistenceWeightsMap>();

  if (self_weight > 1.0 || self_weight < 0) {
    LOG(ERROR) << "self weight must be between 0 and 1, setting to 1";
    self_weight = 1.0;
  }

  for (const Landmark& lmA : (*map)) {
    std::map<uint64_t, double> neighbors;

    // find the 'num_neighbors' closest landmarks within a radius of 'radius'
    //meters which were created in a time less than 'delta time' to the current feature
    for (const Landmark& lmB : (*map)) {

      if (lmB.id == lmA.id) {
        continue;
      }

      double dist  = LandmarkDistance(lmA, lmB);

      if (dist <= radius) {

        if (neighbors.size() < num_neighbors) {

          neighbors[lmB.id] = WeightFromDistance(dist);

        } else {

          for (std::map<uint64_t, double>::iterator it = neighbors.begin(); it != neighbors.end();) {

            if (WeightFromDistance(dist) > it->second) {

              it = neighbors.erase(it);
              neighbors[lmB.id] = WeightFromDistance(dist);
              break;

            }else {
              ++it;
            }
          }
        }
      }
    }

    // now normalize the weights
    double sum = 0.0;
    for(const auto& e : neighbors) {
      sum += e.second;
    }

    if(lmA.id == 26) {
      self_weight = 0.8;
    }

    if(lmA.id == 27 || lmA.id == 28 || lmA.id == 1) {
      self_weight = 0.2;
    }

    for(auto& e : neighbors) {
      e.second = (e.second / (sum)) * (1. - self_weight);
    }

    // assign weight to self
    if (neighbors.empty()) {
      neighbors[lmA.id] = 1.0;
    }else {
      neighbors[lmA.id] = self_weight;
    }
    (*weights_map)[lmA.id] = neighbors;
    for(const auto& e : neighbors) {
      VLOG(1) << fmt::format("Adding neighbor with id: {} and weight {}", e.first, e.second);
    }
    VLOG(1) << fmt::format("Added {} neighbors for lm id: {}", neighbors.size(), lmA.id);
  }

  return weights_map;
}

double DataProvider::LandmarkDistance(const Landmark& lm1, const Landmark& lm2) {
  return (lm1.vec() - lm2.vec()).norm();
}

double DataProvider::WeightFromDistance(double distance) {
  if (distance > 1e-6) {
    return 1.0 / distance;
  }else {
    return 0;
  }
}

bool DataProvider::GetData(RobotData * const data) {
  bool result = false;
  switch (type_) {
  case DataType::Sim:
  {
    result = data_generator_->GetData(data);
  }
    break;
  case DataType::UTIAS:
  {
    result = GetUTIASData(data);
  }
    break;
  case DataType::VicPark:
  {
    //TODO
  }
    break;
  }

  return result;
}

void DataProvider::SetDataType(DataProvider::DataType type) {
  type_ = type;
}

void DataProvider::SetDataType(const std::string type) {
  if (type.compare("sim") == 0) {
    type_ = DataType::Sim;
  } else if (type.compare("vp") == 0) {
    type_ = DataType::VicPark;
  } else if (type.compare("utias") == 0) {
    type_ = DataType::UTIAS;
  }else{
    type_ = DataType::Sim;  // default
  }
}

void DataProvider::LoadData() {
  switch (type_) {
  case DataType::Sim:
  {
    VLOG(1) << "Nothing to do here, simulation data selected.";
  }
    break;
  case DataType::UTIAS:
  {
    VLOG(1) << "Loading UTIAS dataset";
    DataReader::LoadUTIAS(data_file_, &utias_data_);
  }
    break;
  case DataType::VicPark:
  {
    VLOG(1) << "Loading Victoria Park dataset";
    //DataReader::LoadG2o(data_file_);
  }
    break;
  }

  // check if we have real data to load
  //  DataReader::G2oData vic_park_data;
  //  VictoriaParkData vc_data;
  //  if (use_real_data) {
  //    if (FLAGS_data_file.empty()) {
  //      LOG(FATAL) << "Victoria Park data specifiec but no data file passed in.";
  //    } else {
  //      DataReader::LoadG2o(FLAGS_data_file, &vic_park_data);
  //    }
  //  }

}

LandmarkVectorPtr DataProvider::GetUTIASMap() {
  if (type_ != DataType::UTIAS) {
    LOG(ERROR) << " UTIAS data requested but that's not the type that has been set";
    return nullptr;
  }

  if (utias_data_.gt_landmarks.empty()) {
    LOG(ERROR) << " UTIAS Data not loaded, trying to load..";
    LoadData();
  }

  return utias_data_.gt_landmarks_vec;
}

bool DataProvider::GetUTIASData(RobotData * const data) {
  if (type_ != DataType::UTIAS) {
    LOG(ERROR) << " UTIAS data requested but that's not the type that has been set";
    return false;
  }

  if (utias_data_.gt_landmarks.empty()) {
    LOG(ERROR) << " UTIAS Data not loaded, trying to load..";
    LoadData();
  }

  if (utias_data_.keyframes.at(robot_idx_).size() <= current_index_) {
    VLOG(1) << "End of measurements.";
    return false;
  }

  VLOG(2) << " Getting UTIAS data at index: " << current_index_;
  data->index = current_index_;

  DataReader::KeyframeMeasurements::Ptr& kf = utias_data_.keyframes.at(robot_idx_)[current_index_];

  data->timestamp = kf->time;
  VLOG(2) << fmt::format("current ts: {:f} ", data->timestamp);

  data->debug.ground_truth_map = utias_data_.gt_landmarks_vec;

  data->debug.ground_truth_pose = kf->ground_truth_pose;

  data->observations = kf->meas;
  VLOG(2) << fmt::format("{} observations", data->observations.size());

  data->odometry_readings = kf->odometry_meas;
  VLOG(2) << fmt::format("{} odometry readings", data->odometry_readings.size());

  // get odometry measurements from the previous state to this one
  if (current_index_ > 0) {
    data->debug.noisy_pose = OdometryGenerator::PropagateMeasurement(
                               utias_data_.odometry_buffers.at(robot_idx_)->GetRange(
                                 utias_data_.keyframes.at(robot_idx_)[0]->time, data->timestamp),
        utias_data_.keyframes.at(robot_idx_)[0]->ground_truth_pose);
  }else {
    data->debug.noisy_pose = kf->ground_truth_pose;
  }

  data->debug.noisy_observations = kf->meas;
  current_index_++;

  return true;
}

void DataProvider::UpdateOptions(Visualizer& viewer) {
  // update feature detection probabilities
  sim_options_.prob_missed_detection = *(viewer.GetDebugVariables().prob_missed_detect);
  sim_options_.prob_false_positive = *(viewer.GetDebugVariables().prob_false_detect);

  // check if any landmarks need to be removed or changed
  sim_options_.remove_lm_ids.clear();
  sim_options_.change_lm_ids.clear();
  sim_options_.remove_lm_ids = viewer.GetLandmarksToBeRemoved();
  sim_options_.change_lm_ids = viewer.ChangeLandmarks();
}





}  //namespace chameleon
