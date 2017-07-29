// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include <memory>
#include <iostream>

#include <glog/logging.h>

#include "visualizer.h"
#include "motion_generator.h"

/*-----------COMMAND LINE FLAGS-----------------------------------------------*/
DEFINE_bool(display, true, "use viewer (pangolin)");
/*----------------------------------------------------------------------------*/

using namespace elninho;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = 1;

  MotionGenerator::MotionGeneratorOptions path_options;
  path_options.motion_type = MotionGenerator::MotionTypes::Rectangle;
  path_options.num_steps = 100;
  std::unique_ptr<MotionGenerator> motion_generator = util::make_unique<MotionGenerator>(path_options);

  RobotPoseVectorPtr poses =  motion_generator->GenerateMotion();

  Visualizer::ViewerData::Ptr viewer_data = Visualizer::ViewerData::Ptr(new Visualizer::ViewerData());
  viewer_data->robot_poses = poses;

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
