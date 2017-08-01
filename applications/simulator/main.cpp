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

  DataGenerator::DataGeneratorOptions options;  // use default options
  std::unique_ptr<DataGenerator> data_generator = util::make_unique<DataGenerator>(options);

  RobotData data;  // data corresponding to a single timestep

  if (FLAGS_display) {

    VLOG(1) << "Creating visualizer";

    Visualizer::ViewerOptions viewer_options;
    viewer_options.window_name = "Chameleon - ICRA 2018";
    viewer_options.start_running = FLAGS_start_running;

    Visualizer viewer(viewer_options);  // create viewer and run thread

    Visualizer::ViewerData::Ptr viewer_data = std::make_shared<Visualizer::ViewerData>();
    viewer.SetData(viewer_data);  // point to our local viewer data object so everyone is looking at the same data

    bool go = false;
    // until the user requests to finish
    while (!viewer.IsFinished()) {

      //  feed the viewer some data so we have something to display
      go = viewer.IsStepping() || viewer.IsRunning();
      if (go && !viewer.IsRunning()) {
        // not running continuously, set step to false so we pause after 1 pose
        viewer.SetStepping(false);
      }

      if (go) {
        // Get some data
        if (data_generator->GetRobotData(&data)) {
          // and display it
          viewer_data->AddData(data);
          viewer.AddTimesteps({size_t(data.timestamp)});  // add the current timestep to the display
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }



  return 0;
}
