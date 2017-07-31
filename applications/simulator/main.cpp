// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include <memory>
#include <iostream>

#include <glog/logging.h>

#include "chameleon/viewer/visualizer.h"
#include "chameleon/data_generator.h"
#include "fmt/printf.h"

/*-----------COMMAND LINE FLAGS-----------------------------------------------*/
DEFINE_bool(display, true, "use viewer (pangolin)");
/*----------------------------------------------------------------------------*/

using namespace chameleon;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = 1;

  DataGenerator::DataGeneratorOptions options;  // use default options
  options.path_options.motion_type = PathGenerator::PathTypes::Rectangle;
  std::unique_ptr<DataGenerator> data_generator = util::make_unique<DataGenerator>(options);

  SimData simulated_data;
  data_generator->GenerateSimulatedData(&simulated_data);

  Visualizer::ViewerData::Ptr viewer_data = std::make_shared<Visualizer::ViewerData>();
  viewer_data->ground_truth_robot_poses = simulated_data.debug.ground_truth_poses;
  viewer_data->noisy_robot_poses = simulated_data.debug.noisy_poses;
  viewer_data->ground_truth_map = simulated_data.debug.ground_truth_map;

  if (FLAGS_display) {
    VLOG(1) << "Creating visualizer";
    Visualizer::ViewerOptions viewer_options;
    viewer_options.window_name = "Change Detection Simulator";
    Visualizer viewer(viewer_options);  // create viewer and run thread

    viewer.SetData(viewer_data);

    while (!viewer.IsFinished()) {
      usleep(50000);
    }
  }



  return 0;
}
