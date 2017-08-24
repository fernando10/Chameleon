// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include <memory>
#include <iostream>

#include <glog/logging.h>

#include "chameleon/viewer/visualizer.h"
#include "chameleon/data_generator.h"
#include "chameleon/data_provider.h"
#include "chameleon/estimator.h"
#include "fmt/printf.h"
#include "chameleon/data_reader.h"

/*-----------COMMAND LINE FLAGS-----------------------------------------------*/
DEFINE_bool(display, true, "use viewer (pangolin)");
DEFINE_bool(start_running, false, "start running immediately");
DEFINE_bool(add_observations, true, "add landmark observation/estimation");
DEFINE_int32(num_steps, 200, " number of steps to take");
DEFINE_bool(compute_lm_covariance, false, "compute landmark covariance");
DEFINE_bool(provide_map, true, "provide pre-built map to estimator");
DEFINE_bool(print_optimization_full_summary, false, "print full summary");
DEFINE_bool(print_optimization_brief_summary, false, "print brief summary");
DEFINE_bool(persistence_filter, false, "use persistence filter");
DEFINE_bool(weigh_lm_by_persistence, false, "weigh lm uncertainty by persistence");
DEFINE_double(huber_loss, 1.0, " huber loss");
DEFINE_int32(delayed_lm_init, 10, " delayed lm initalization");
DEFINE_string(data_file, "", "data file for dataset");
DEFINE_string(data_type, "sim", "data type: sim, vp (victoria park) or utias");
DEFINE_string(data_association, "known", "data association type: known, IC or JCBB");
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

  ///////////////////////////////////////
  // Setup Data Provider so we have some data
  //////////////////////////////////////
  DataProvider data_provider(FLAGS_data_type, FLAGS_data_file);
  DataGenerator::DataGeneratorOptions& sim_options = data_provider.SimulationOptions();
  sim_options.path_options.num_steps = FLAGS_num_steps;
  sim_options.path_options.initial_position = RobotPose(0, Eigen::Vector2d::Zero());

  //////////////////////////////////////
  // Setup estimator
  //////////////////////////////////////
  chameleon::ceres::Estimator::EstimatorOptions estimator_options;
  estimator_options.print_full_summary = true;
  if (FLAGS_data_association.compare("known") == 0) {
    estimator_options.data_association_strategy = DataAssociation::DataAssociationType::Known;
  } else if (FLAGS_data_association.compare("IC") == 0) {
    estimator_options.data_association_strategy = DataAssociation::DataAssociationType::IC;
  }
  estimator_options.add_observations = FLAGS_add_observations;
  estimator_options.provide_map = FLAGS_provide_map;
  estimator_options.weigh_landmarks_by_persistence = FLAGS_weigh_lm_by_persistence;
  estimator_options.print_brief_summary = FLAGS_print_optimization_brief_summary;
  estimator_options.print_full_summary = FLAGS_print_optimization_full_summary;
  estimator_options.compute_landmark_covariance = FLAGS_compute_lm_covariance;
  estimator_options.ceres_options.minimizer_progress_to_stdout = FLAGS_print_optimization_full_summary;
  estimator_options.huber_loss_a = FLAGS_huber_loss;
  estimator_options.filter_options.use_persistence_filter = FLAGS_persistence_filter;
  estimator_options.delayed_initalization_num = FLAGS_delayed_lm_init;
  std::unique_ptr<chameleon::ceres::Estimator> SLAM = util::make_unique<chameleon::ceres::Estimator>(estimator_options);
  EstimatedData estimator_results;  // for collecting the latest updates form the estimator

  //  if (FLAGS_provide_map) {
  //    SLAM->SetMap(data_generator->GetNoisyMap());
  //  }

  ////////////////////////////////////
  // Get Data -> Estimate -> Display
  ////////////////////////////////////
  if (FLAGS_display) {
    Visualizer::ViewerOptions viewer_options;
    viewer_options.window_name = "Chameleon - TRI Summer 2017";
    viewer_options.start_running = FLAGS_start_running;

    Visualizer viewer(viewer_options);  // create viewer and run thread

    Visualizer::ViewerData::Ptr viewer_data = std::make_shared<Visualizer::ViewerData>();
    viewer.SetData(viewer_data);  // point to our local viewer data object so everyone is looking at the same data

    bool go = false;
    // until the user requests to finish
    while (!viewer.IsFinished()) {

      if (viewer.IsResetRequested()) {
        LOG(INFO) << " Reseting...";
        data_provider.Reset();  // reset data provider
        SLAM->Reset();  // reset slam system
        //        if (estimator_options.provide_map) {
        //          SLAM->SetMap(data_generator->GetNoisyMap());
        //        }

        // reset viewer
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

        // pass along any user input to the data provider
        data_provider.UpdateOptions(viewer);

        // Get some data
        bool data_success = false;
        RobotData data;
        data_success = data_provider.GetData(&data);

        if (data_success) {

          // display data
          viewer_data->AddData(data);

          if (*(viewer.GetDebugVariables().do_SLAM)) {
            estimator_options.filter_options.P_F = *(viewer.GetDebugVariables().prob_false_detect);
            estimator_options.filter_options.P_M = *(viewer.GetDebugVariables().prob_missed_detect);
            SLAM->AddData(data);
            SLAM->SetLocalizationMode(*(viewer.GetDebugVariables().do_Localization));
            // solve is currently batch synchronous....TBD if needs to be threaded
            SLAM->Solve();
            SLAM->GetEstimationResult(&estimator_results);
            viewer_data->AddData(estimator_results);
          }

          viewer.AddTimesteps({size_t(data.timestamp)});  // add the current timestep to the display

        } else {
          LOG(ERROR) << "Unable to get data.";
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  LOG(INFO) << " Bye";
  return 0;
}  
