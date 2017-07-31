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
DEFINE_bool(start_running, true, " start running immediately");
/*----------------------------------------------------------------------------*/

using namespace chameleon;
bool is_running = false;
bool is_stepping = false;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = 1;

  DataGenerator::DataGeneratorOptions options;  // use default options
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
    viewer_options.start_running = FLAGS_start_running;
    Visualizer viewer(viewer_options);  // create viewer and run thread
    viewer.SetData(viewer_data);  // point to our local viewer data object so everyone is looking at the same data

    bool go = false;
    size_t num_poses = simulated_data.times.size();
    size_t pose_idx = 0;
    // until the user requests to finish
    while (!viewer.IsFinished()) {

      //  feed the viewer some data so we have something to display
      go = viewer.IsStepping() || viewer.IsRunning();
      if (go && !viewer.IsRunning()) {
        // not running continuously, set step to false so we pause after 1 pose
        viewer.SetStepping(false);
      }

      if (go) {
        if (pose_idx < num_poses) {
          VLOG(3) << fmt::format("Adding timestep: {}", pose_idx);
          viewer.AddTimesteps({pose_idx});  // add the current timestep to the display
          pose_idx++;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }



  return 0;
}
