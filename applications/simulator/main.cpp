// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include <memory>
#include <iostream>

#include <glog/logging.h>

#include "chameleon/viewer/visualizer.h"
#include "chameleon/data_generator.h"
#include "chameleon/estimator.h"
#include "fmt/printf.h"

/*-----------COMMAND LINE FLAGS-----------------------------------------------*/
DEFINE_bool(display, true, "use viewer (pangolin)");
DEFINE_bool(start_running, false, "start running immediately");
DEFINE_bool(do_slam, true, "do state estimation");
/*----------------------------------------------------------------------------*/

using namespace chameleon;

////////////////////////////////
/// UI variables
///////////////////////////////
bool is_running = false;  // continuous run
bool is_stepping = false;  // take only one step and pause

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = 1;

  // Setup Data Generator so we have some simulated data
  DataGenerator::DataGeneratorOptions sim_options;  // use default options
  sim_options.path_options.num_steps = 1000;
  std::unique_ptr<DataGenerator> data_generator = util::make_unique<DataGenerator>(sim_options);

  RobotData simulator_data;  // data corresponding to a single timestep

  // Setup estimator
  chameleon::ceres::Estimator::EstimatorOptions estimator_options;
  estimator_options.print_full_summary = true;
  estimator_options.data_association_strategy = DataAssociation::DataAssociationType::Known;
  std::unique_ptr<chameleon::ceres::Estimator> SLAM = util::make_unique<chameleon::ceres::Estimator>(estimator_options);

  if (FLAGS_display) {
    Visualizer::ViewerOptions viewer_options;
    viewer_options.window_name = "Chameleon - ICRA 2018";
    viewer_options.start_running = FLAGS_start_running;

    Visualizer viewer(viewer_options);  // create viewer and run thread

    Visualizer::ViewerData::Ptr viewer_data = std::make_shared<Visualizer::ViewerData>();
    viewer.SetData(viewer_data);  // point to our local viewer data object so everyone is looking at the same data

    EstimatedData estimator_results;  // for collecting the latest updates form the estimator

    bool go = false;
    // until the user requests to finish
    while (!viewer.IsFinished()) {

      if (viewer.IsResetRequested()) {
        LOG(INFO) << " Reseting...";
        data_generator->Reset();
        SLAM->Reset();
        viewer_data = std::make_shared<Visualizer::ViewerData>();
        viewer.SetData(viewer_data);
        viewer.SetReset();
      }

      go = viewer.IsStepping() || viewer.IsRunning();
      if (go && !viewer.IsRunning()) {
        // not running continuously, set step to false so we pause after 1 pose
        viewer.SetStepping(false);
      }

      if (go) {
        // Get some data
        if (data_generator->GetRobotData(&simulator_data)) {

          // display debug data
          viewer_data->AddData(simulator_data);

          if (FLAGS_do_slam) {
            SLAM->AddData(simulator_data);
            // solve is currently synchronous....TBD if needs to be threaded
            SLAM->Solve();
            SLAM->GetEstimationResult(&estimator_results);
            viewer_data->AddData(estimator_results);
          }

          viewer.AddTimesteps({size_t(simulator_data.timestamp)});  // add the current timestep to the display

        }else {
          LOG(ERROR) << "Unable to get data.";
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }



  return 0;
}
