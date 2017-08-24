// Copyright 2017 Toyota Research Institute.  All rights reserved.
//

#include "chameleon/data_provider.h"
#include "chameleon/util.h"
namespace chameleon
{

DataProvider::DataProvider(std::string data_type, std::string data_file) {
  SetDataType(data_type);
  data_file_ = data_file;
  data_generator_ = util::make_unique<DataGenerator>(sim_options_);

  LoadData();
}

void DataProvider::Reset() {
 data_generator_->Reset();
}

bool DataProvider::GetData(RobotData * const data) {
  bool result = false;
  switch (type_) {
  case DataType::Sim:
    result = data_generator_->GetData(data);
    break;
  case DataType::UTIAS:
    //TODO
    break;
  case DataType::VicPark:
    //TODO
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
    VLOG(1) << "Nothing to do here, simulation data selected.";
    break;
  case DataType::UTIAS:
    VLOG(1) << "Loading UTIAS dataset";
    //TODO
    break;
  case DataType::VicPark:
    VLOG(1) << "Loading Victoria Park dataset";
    //TODO
    //DataReader::LoadG2o(data_file_);
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
