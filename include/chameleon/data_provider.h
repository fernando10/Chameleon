// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include "chameleon/types.h"
#include "chameleon/data_provider_base.h"
#include "chameleon/data_generator.h"
#include "chameleon/data_reader.h"
#include "chameleon/viewer/visualizer.h"

namespace chameleon
{

class DataProvider : DataProviderBase {
public:
  enum class DataType {
    Sim,
    VicPark,
    UTIAS
  };

  DataProvider(std::string data_type, std::string data_file);

  bool GetData(RobotDataVec* const data);
  bool GetData(RobotData* const data) override;

  LandmarkVectorPtr GetPriorMap();

  void LoadData(const std::string path, DataType type);

  DataGenerator::DataGeneratorOptions& SimulationOptions() {
    return sim_options_;
  }

  void UpdateOptions(Visualizer& viewer);
  void SetDataType(DataProvider::DataType type);
  void SetDataType(const std::string type);

  void Reset();


  FeaturePersistenceWeightsMapPtr BuildFeaturePersistenceAssociationMatrix(double self_weight = 0.7, double num_neighbors = 5, double radius = 2,
                                                                           double delta_time = 1);
  FeaturePersistenceWeightsMapPtr BuildFeaturePersistenceAssociationMatrix(const LandmarkVectorPtr& map,
                                                                           double self_weight = 0.7,
                                                                           double num_neighbors = 5, double radius = 2,
                                                                           double delta_time = 1);

  size_t num_robots = 1;


private:
  void LoadData();
  bool GetUTIASData(std::vector<RobotData> * const data, size_t num_robots = 1);
  bool GetUTIASData(RobotData * const data, bool increment = true, size_t robot_idx = 1);
  LandmarkVectorPtr GetUTIASMap();
  LandmarkVectorPtr GetVicParkMap();
  double WeightFromDistance(double distance);
  double LandmarkDistance(const Landmark& lm1, const Landmark& lm2);


  DataType type_;
  DataGenerator::DataGeneratorOptions sim_options_;
  std::unique_ptr<DataGenerator> data_generator_;
  RobotData simulator_data_;
  std::string data_file_;
  DataReader::UTIASData utias_data_;
  DataReader::G2oData vic_park_data_;
  uint64_t current_index_;
};
}  //  namespace chameleon

