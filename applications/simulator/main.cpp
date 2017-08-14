// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include <memory>
#include <iostream>

#include <glog/logging.h>

#include "chameleon/viewer/visualizer.h"
#include "chameleon/data_generator.h"
#include "chameleon/estimator.h"
#include "fmt/printf.h"
#include "chameleon/data_reader.h"

/*-----------COMMAND LINE FLAGS-----------------------------------------------*/
DEFINE_bool(display, true, "use viewer (pangolin)");
DEFINE_bool(start_running, false, "start running immediately");
DEFINE_bool(add_observations, true, "add landmark observation/estimation");
DEFINE_int32(num_steps, 200, " number of steps to take");
DEFINE_bool(compute_lm_covariance, false, "compute landmark covariance");
DEFINE_bool(print_optimization_full_summary, false, "print full summary");
DEFINE_bool(print_optimization_brief_summary, false, "print brief summary");
DEFINE_bool(persistence_filter, false, "use persistence filter");
DEFINE_double(huber_loss, 1.0, " huber loss");
DEFINE_int32(delayed_lm_init, 10, " delayed lm initalization");
DEFINE_string(data_file, "", "data file for victoria park g2o dataset");
DEFINE_string(data_type, "sim", "data type: sim or vp (victoria park)");
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

  bool use_sim_data = FLAGS_data_type.compare("sim") == 0;
  bool use_real_data = FLAGS_data_type.compare("vp") == 0;


  ///////////////////////////////////////
  // Setup Data Generator so we have some simulated data
  //////////////////////////////////////
  DataGenerator::DataGeneratorOptions sim_options;  // use default options
  sim_options.path_options.num_steps = FLAGS_num_steps;
  sim_options.path_options.initial_position = RobotPose(0, Eigen::Vector2d::Zero());
  std::unique_ptr<DataGenerator> data_generator = util::make_unique<DataGenerator>(sim_options);
  RobotData simulator_data;  // data corresponding to a single timestep

  // check if we have real data to load
  DataReader::G2oData vic_park_data;
  VictoriaParkData vc_data;
  if (use_real_data) {
    if (FLAGS_data_file.empty()) {
      LOG(FATAL) << "Victoria Park data specifiec but no data file passed in.";
    } else {
      DataReader::LoadG2o(FLAGS_data_file, &vic_park_data);
    }
  }

  //////////////////////////////////////
  // Setup estimator
  //////////////////////////////////////
  chameleon::ceres::Estimator::EstimatorOptions estimator_options;
  estimator_options.print_full_summary = true;
  estimator_options.data_association_strategy = DataAssociation::DataAssociationType::Known;
  estimator_options.add_observations = FLAGS_add_observations;
  estimator_options.print_brief_summary = FLAGS_print_optimization_brief_summary;
  estimator_options.print_full_summary = FLAGS_print_optimization_full_summary;
  estimator_options.compute_landmark_covariance = FLAGS_compute_lm_covariance;
  estimator_options.ceres_options.minimizer_progress_to_stdout = FLAGS_print_optimization_full_summary;
  estimator_options.huber_loss_a = FLAGS_huber_loss;
  estimator_options.filter_options.use_persistence_filter = FLAGS_persistence_filter;
  estimator_options.delayed_initalization_num = FLAGS_delayed_lm_init;
  std::unique_ptr<chameleon::ceres::Estimator> SLAM = util::make_unique<chameleon::ceres::Estimator>(estimator_options);
  EstimatedData estimator_results;  // for collecting the latest updates form the estimator

  ////////////////////////////////////
  // Get Data -> Estimate -> Display
  ////////////////////////////////////
  if (FLAGS_display) {
    Visualizer::ViewerOptions viewer_options;
    viewer_options.window_name = "Chameleon - ICRA 2018";
    viewer_options.start_running = FLAGS_start_running;

    Visualizer viewer(viewer_options);  // create viewer and run thread

    Visualizer::ViewerData::Ptr viewer_data = std::make_shared<Visualizer::ViewerData>();
    viewer.SetData(viewer_data);  // point to our local viewer data object so everyone is looking at the same data

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

        // update feature detection probabilities
        sim_options.prob_missed_detection = *(viewer.GetDebugVariables().prob_missed_detect);
        sim_options.prob_false_positive = *(viewer.GetDebugVariables().prob_false_detect);

        // check if any landmarks need to be removed
        sim_options.remove_lm_ids = viewer.GetLandmarksToBeRemoved();

        // Get some data
        bool data_success = false;
        if (use_sim_data) {
          VLOG(1) << "Getting simulated data.";
          data_success = data_generator->GetRobotData(&simulator_data);
        } else if (use_real_data) {
          VLOG(1) << "Getting victoria park data.";
          //data_success = vic_park_data.GetData();
        }

        if (data_success) {

          // display debug data
          if (use_sim_data) {
            viewer_data->AddData(simulator_data);
          } else if (use_real_data) {
            viewer_data->AddData(vic_park_data);
          }

          if (*(viewer.GetDebugVariables().do_SLAM)) {
            estimator_options.filter_options.P_F = *(viewer.GetDebugVariables().prob_false_detect);
            estimator_options.filter_options.P_M = *(viewer.GetDebugVariables().prob_missed_detect);
            SLAM->AddData(simulator_data);
            SLAM->SetLocalizationMode(*(viewer.GetDebugVariables().do_Localization));
            // solve is currently batch synchronous....TBD if needs to be threaded
            SLAM->Solve();
            SLAM->GetEstimationResult(&estimator_results);
            viewer_data->AddData(estimator_results);
          }

          if (use_sim_data) {
            viewer.AddTimesteps({size_t(simulator_data.timestamp)});  // add the current timestep to the display
          } else {
            std::vector<size_t> ts;
            viewer.AddTimesteps(ts);
          }

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
