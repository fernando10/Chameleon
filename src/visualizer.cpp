// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include <thread>
#include <mutex>
#include "chameleon/viewer/visualizer.h"
#include "fmt/format.h"
#include <glog/logging.h>

namespace chameleon
{

Visualizer::Visualizer(const ViewerOptions& options):
  options_(options), single_step_(false), running_(options.start_running) {

  VLOG(1) << fmt::format("Starting viewer thread. Running: {}, Stepping: {}", running_, single_step_);
  // start displaying things
  viewer_thread_ = std::unique_ptr<std::thread>(new std::thread(&Visualizer::Run, this));
}

void Visualizer::SwitchProjection(ProjectionMatrixTypes type) {
  if (type == ProjectionMatrixTypes::Perspective) {
    VLOG(1) << "Switching to perspective projection";
    gui_vars_.camera.SetProjectionMatrix(pangolin::ProjectionMatrix(options_.window_width, options_.window_height,
                                                                    420, 420, options_.window_width/2., options_.window_height/2. , 0.01, 5000));
  } else if (type == ProjectionMatrixTypes::Orthographic) {
    VLOG(1) << "Switching to orthographic projection";
    gui_vars_.camera.SetProjectionMatrix(pangolin::ProjectionMatrixOrthographic(-20, 20,
                                                                                -20, 20, -50, 1000));
  }
}

void Visualizer::AddObjectsToSceneGraph() {
  // create scene graph objects
  gui_vars_.light = util::make_unique<SceneGraph::GLLight>(10, 10, -1000);
  gui_vars_.scene_graph.AddChild(gui_vars_.light.get());
  gui_vars_.dynamic_grid = util::make_unique<SceneGraph::GLDynamicGrid>();
  gui_vars_.scene_graph.AddChild(gui_vars_.dynamic_grid.get());

  // add robot path to the scene graph (gt and noisy)
  gui_vars_.gt_robot_path = util::make_unique<GLPathAbs>();
  gui_vars_.scene_graph.AddChild(gui_vars_.gt_robot_path.get());
  gui_vars_.gt_robot_path->SetColor(0, 0, 1);

  gui_vars_.noisy_robot_path = util::make_unique<GLPathAbs>();
  gui_vars_.scene_graph.AddChild(gui_vars_.noisy_robot_path.get());
  gui_vars_.noisy_robot_path->SetColor(1, 0, 0);

  // and the ground truth map
  gui_vars_.ground_truth_map = util::make_unique<GLMap>();
  gui_vars_.scene_graph.AddChild(gui_vars_.ground_truth_map.get());

  // and the observations
  gui_vars_.ground_truth_observations = util::make_unique<GLObservations>();
  gui_vars_.scene_graph.AddChild(gui_vars_.ground_truth_observations.get());
  gui_vars_.ground_truth_observations->SetColor(1, 1, 1);  // white
  gui_vars_.noisy_observations = util::make_unique<GLObservations>();
  gui_vars_.scene_graph.AddChild(gui_vars_.noisy_observations.get());
  gui_vars_.noisy_observations->SetColor(0.6, 0.1, 0.1);  // dark red
  gui_vars_.noisy_observations->SetLineWidth(1.7);

  // data associations
  gui_vars_.data_associations = util::make_unique<GLDataAssociations>();
  gui_vars_.scene_graph.AddChild(gui_vars_.data_associations.get());

  // add estimated quantities
  gui_vars_.estimated_robot_path = util::make_unique<GLPathAbs>();
  gui_vars_.scene_graph.AddChild(gui_vars_.estimated_robot_path.get());
  gui_vars_.estimated_robot_path->SetColor(0, 1, 0);

  gui_vars_.estimated_map = util::make_unique<GLMap>();
  gui_vars_.estimated_map->SetColor(1, 0, 1);  // magenta
  gui_vars_.estimated_map->SetShowPersistenceLabels(false);
  gui_vars_.scene_graph.AddChild(gui_vars_.estimated_map.get());

  remove_lm_idx_ = 18;  // TEMP, remove this
}

void Visualizer::ResetSceneGraph() {
  gui_vars_.scene_graph.Clear();
  AddObjectsToSceneGraph();
}

void Visualizer::InitGui() {
  VLOG(1) << "Initializing GUI.";
  std::unique_lock<std::mutex> lock(status_mutex_);
  // Create a window.
  pangolin::CreateWindowAndBind(options_.window_name,
                                options_.window_width,
                                options_.window_height);

  // setup camera
  gui_vars_.camera.SetModelViewMatrix(pangolin::ModelViewLookAt(10, 3, -15, 10, 0, 0, pangolin::AxisNegZ));
  gui_vars_.camera.SetProjectionMatrix(pangolin::ProjectionMatrix(options_.window_width, options_.window_height,
                                                                  420, 420, options_.window_width/2., options_.window_height/2. , 0.01, 5000));

  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  // Reset background color to black.
  glClearColor(0, 0, 0, 1);

  AddObjectsToSceneGraph();

  // create view that will contain the robot/landmarks
  gui_vars_.world_view_ptr.reset(&pangolin::CreateDisplay()
                                 .SetAspect(-(float)options_.window_width/(float)options_.window_height)
                                 .SetBounds(0.25, 1.0, /*pangolin::Attach::Pix(options_.panel_size)*/0.0, 1.0));

  // create view for the controls panel
  gui_vars_.panel_view_ptr.reset(&pangolin::CreatePanel("ui").
                                 SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(options_.panel_size)));

  // create handler to allow user input to update modelview matrix and flow through to the scene graph
  gui_vars_.handler.reset(new SceneGraph::HandlerSceneGraph(gui_vars_.scene_graph, gui_vars_.camera,
                                                            pangolin::AxisNegZ));

  gui_vars_.world_view_ptr->SetHandler(gui_vars_.handler.get());
  gui_vars_.world_view_ptr->SetDrawFunction(SceneGraph::ActivateDrawFunctor(gui_vars_.scene_graph,
                                                                            gui_vars_.camera));

  gui_vars_.log_ptr.reset(new pangolin::DataLog());
  std::vector<std::string> data_labels;
  data_labels.push_back("P(X=1 | Z)");
  gui_vars_.log_ptr->SetLabels(data_labels);

  gui_vars_.plotter_ptr.reset(new pangolin::Plotter(gui_vars_.log_ptr.get()));
  pangolin::XYRange<float> range(0.f, 800.f, 0.f, 1.f);
  gui_vars_.plotter_ptr->SetDefaultView(range);
  gui_vars_.plotter_ptr->SetViewSmooth(range);
  gui_vars_.plotter_ptr->ToggleTracking();

  // create a container view in case we want to have multiple views on the rigth side (maybe add some plots, etc)
  gui_vars_.multi_view_ptr.reset(&pangolin::Display("multi")
                                 .SetBounds(0.0, 1.0, pangolin::Attach::Pix(options_.panel_size), 1.0)
                                 .SetLayout(pangolin::LayoutVertical));

  // add the world view
  gui_vars_.multi_view_ptr->AddDisplay(*(gui_vars_.world_view_ptr));
  gui_vars_.multi_view_ptr->AddDisplay(*(gui_vars_.plotter_ptr));



  ////////////////////////////////////////////////////
  /// KEYBINDINGS
  ///////////////////////////////////////////////////
  pangolin::RegisterKeyPressCallback('p', std::bind(&Visualizer::SwitchProjection,
                                                    this, ProjectionMatrixTypes::Perspective));
  pangolin::RegisterKeyPressCallback('o', std::bind(&Visualizer::SwitchProjection,
                                                    this, ProjectionMatrixTypes::Orthographic));
  pangolin::RegisterKeyPressCallback(' ' , [&]() { running_ = !running_; });
  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT,
                                     [&]() { single_step_ = true; });

  pangolin::RegisterKeyPressCallback('r', [&]() { VLOG(1) << "removing landmark id: " << remove_lm_idx_;
    landmarks_to_be_removed_.push_back(remove_lm_idx_++); });

  pangolin::RegisterKeyPressCallback('s', [&]() { *gui_vars_.ui.do_SLAM = !(*gui_vars_.ui.do_SLAM); });
  pangolin::RegisterKeyPressCallback('l', [&]() { *gui_vars_.ui.do_Localization = !(*gui_vars_.ui.do_Localization); });
  pangolin::RegisterKeyPressCallback('m', [&]() { VLOG(1) << "changing landmarks... ";
    change_landmarks_ = true; });


  ////////////////////////////////////////////////////
  /////UI VARIABLES
  /// ////////////////////////////////////////////////
  gui_vars_.ui.reset = util::make_unique<pangolin::Var<bool>>("ui.Reset", false, false);
  gui_vars_.ui.show_gt = util::make_unique<pangolin::Var<bool>>("ui.Show_ground_truth", false, true);
  gui_vars_.ui.show_data_assoc = util::make_unique<pangolin::Var<bool>>("ui.Show_data_assoc.", false, true);
  gui_vars_.ui.show_observations = util::make_unique<pangolin::Var<bool>>("ui.Show_observations", true, true);
  gui_vars_.ui.show_gt_observations = util::make_unique<pangolin::Var<bool>>("ui.Show_gt_observations", false, true);
  gui_vars_.ui.show_landmarks = util::make_unique<pangolin::Var<bool>>("ui.Show_landmarks", false, true);
  gui_vars_.ui.show_odometry = util::make_unique<pangolin::Var<bool>>("ui.Show_odometry", false, true);
  gui_vars_.ui.show_estimated = util::make_unique<pangolin::Var<bool>>("ui.Show_estimated", true, true);
  gui_vars_.ui.show_variance = util::make_unique<pangolin::Var<bool>>("ui.Show_variance", false, true);
  gui_vars_.ui.do_SLAM = util::make_unique<pangolin::Var<bool>>("ui.Do_SLAM", true, true);
  gui_vars_.ui.do_Localization = util::make_unique<pangolin::Var<bool>>("ui.Localization", false, true);
  gui_vars_.ui.show_prob_labels = util::make_unique<pangolin::Var<bool>>("ui.Show_posterior", false, true);
  gui_vars_.ui.show_lm_ids = util::make_unique<pangolin::Var<bool>>("ui.Show_lm_ids", false, true);
  gui_vars_.ui.color_lms = util::make_unique<pangolin::Var<bool>>("ui.Color_landmarks", false, true);
  gui_vars_.ui.draw_groups = util::make_unique<pangolin::Var<bool>>("ui.Draw_Groups", false, true);

  gui_vars_.ui.plot_idx = util::make_unique<pangolin::Var<int>>("debug.plot_lm", 17);
  *gui_vars_.ui.plot_idx = 17;


  gui_vars_.ui.prob_missed_detect = util::make_unique<pangolin::Var<double>>("ui.Prob. Missed Detect.", 0.0, 1.0);
  *gui_vars_.ui.prob_missed_detect = 0.2;
  gui_vars_.ui.prob_false_detect = util::make_unique<pangolin::Var<double>>("ui.Prob. False Detect", 0.0, 1.0);
  *gui_vars_.ui.prob_false_detect = 0.2;
}

std::vector<uint64_t> Visualizer::GetLandmarksToBeRemoved() {
  std::vector<uint64_t> ret = landmarks_to_be_removed_;
  landmarks_to_be_removed_.clear();
  return ret;
}

std::vector<uint64_t> Visualizer::ChangeLandmarks() {
  if(change_landmarks_) {
    std::vector<uint64_t> ret {1, 2, 27, 28};
    change_landmarks_ = false;
    return ret;
  }else{
    return std::vector<uint64_t>();
  }
}


void Visualizer::Run() {

  InitGui();  // the gui needs to be initialized in the same thread

  // Viewer loop.
  while (!pangolin::ShouldQuit() || CheckFinish()) {

    // clear whole screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // check toggles
    gui_vars_.ground_truth_map->SetVisible(*gui_vars_.ui.show_landmarks);
    gui_vars_.ground_truth_observations->SetVisible(*gui_vars_.ui.show_gt_observations);
    gui_vars_.data_associations->SetVisible(*gui_vars_.ui.show_data_assoc);
    gui_vars_.noisy_observations->SetVisible(*gui_vars_.ui.show_observations);
    gui_vars_.noisy_robot_path->SetVisible(*gui_vars_.ui.show_odometry);
    gui_vars_.gt_robot_path->SetVisible(*gui_vars_.ui.show_gt);
    gui_vars_.estimated_robot_path->SetVisible(*gui_vars_.ui.show_estimated);
    gui_vars_.estimated_map->SetVisible(*gui_vars_.ui.show_estimated);
    gui_vars_.estimated_map->SetShowPersistenceLabels(*gui_vars_.ui.show_prob_labels);
    gui_vars_.estimated_map->SetShowLandmarkId(*gui_vars_.ui.show_lm_ids);
    gui_vars_.estimated_map->SetColorBasedOnPersistence(*gui_vars_.ui.color_lms);
    gui_vars_.estimated_map->SetDrawGroups(*gui_vars_.ui.draw_groups);
    gui_vars_.estimated_map->SetShowVariance(*gui_vars_.ui.show_variance);


    if (pangolin::Pushed(*gui_vars_.ui.reset) ) {
      RequestReset();
    }

    pangolin::FinishFrame();


    // Pause for 1/60th of a second
    std::this_thread::sleep_for(std::chrono::microseconds(1000 / 60));
  }
  SetFinish();
}

void Visualizer::SetData(ViewerData::Ptr data) {
  std::unique_lock<std::mutex> lock(data_mutex_);
  data_ = data;
}

void Visualizer::AddLandmarks() {

  // update the ground truth landmarks
  if (data_->ground_truth_map != nullptr) {
    gui_vars_.ground_truth_map->GetMapRef().clear();
    for (const auto& lm : *(data_->ground_truth_map)) {
      gui_vars_.ground_truth_map->GetMapRef().push_back(Landmark(lm));
    }
  }
  // and add the estimated landmarks (update every time since they can change location at every timestep)
  gui_vars_.estimated_map->Clear();
  for (const auto& e : data_->estimated_landmarks) {
    gui_vars_.estimated_map->GetMapRef().push_back(Landmark(*(e.second)));
  }
}

void Visualizer::UpdatePlotters() {
  // get the last landmark for now
  if (!data_->estimated_landmarks.empty() && data_->estimated_landmarks.find(*gui_vars_.ui.plot_idx) !=
      data_->estimated_landmarks.end()) {
    gui_vars_.log_ptr->Log(data_->estimated_landmarks.at(*gui_vars_.ui.plot_idx)->persistence_prob);
  }
}

bool Visualizer::AddIndices(std::vector<size_t> indices) {
  std::unique_lock<std::mutex>(data_mutex_);

  AddLandmarks();
  UpdatePlotters();

  for (const size_t& ts : indices) {
    if (data_ == nullptr) {
      LOG(ERROR) << fmt::format("Requested to add timestep: {} but viewer has no valid data pointer...", ts);
      return false;
    }
    // add the robot pose at the current time
    if (data_->ground_truth_robot_poses != nullptr && data_->ground_truth_robot_poses->size() > ts) {
      // data exsits, add this pose to the display
      RobotPose& robot = data_->ground_truth_robot_poses->at(ts);
      std::vector<Sophus::SE2d>& poses_path_ref = gui_vars_.gt_robot_path->GetPathRef();
      poses_path_ref.push_back(robot.pose);
      VLOG(3) << fmt::format("Added pose to path at: {}, {}", robot.pose.translation().x(), robot.pose.translation().y());

      // add the ground truth landmark observations
      if (data_->ground_truth_observation_map.find(ts) != data_->ground_truth_observation_map.end()) {
        // get the observations for this timestep
        const RangeFinderObservationVector& gt_observations = data_->ground_truth_observation_map.at(ts);
        gui_vars_.ground_truth_observations->SetPoseAndObservations(robot, gt_observations);
      }

      // add the noisy landmark observations
      if (data_->noisy_observation_map.find(ts) != data_->noisy_observation_map.end()) {
        // get the observations for this timestep
        const RangeFinderObservationVector& noisy_observations = data_->noisy_observation_map.at(ts);
        RobotPose obs_pose = robot;
        if (data_->estimated_poses.size() > ts) {
          obs_pose = data_->estimated_poses.rbegin()->second->robot;
        }
        gui_vars_.noisy_observations->SetPoseAndObservations(obs_pose, noisy_observations);
      }

    } else {
      LOG(ERROR) << fmt::format("Error adding robot pose at timestep: {}, either data is null or index does not exist.", ts);
    }

    // lets also add the noisy robot poses at the current time
    if (data_->noisy_robot_poses != nullptr && data_->noisy_robot_poses->size() > ts) {
      // data exsits, add this pose to the display
      RobotPose& noisy_robot = data_->noisy_robot_poses->at(ts);
      std::vector<Sophus::SE2d>& poses_path_ref = gui_vars_.noisy_robot_path->GetPathRef();
      poses_path_ref.push_back(noisy_robot.pose);
      VLOG(3) << fmt::format("Added noisy pose to path at: {}, {}", noisy_robot.pose.translation().x(), noisy_robot.pose.translation().y());
    } else {
      LOG(ERROR) << fmt::format("Error adding noisy robot pose at timestep: {}, either data is null or index does not exist.", ts);
    }

    // and the estimated quantities
    if (data_->estimated_poses.size() > ts) {
      std::vector<Sophus::SE2d>& poses_path_ref = gui_vars_.estimated_robot_path->GetPathRef();

      // remove all states and re-add (for now)
      gui_vars_.estimated_robot_path->Clear();
      for (const auto& state : data_->estimated_poses) {
        poses_path_ref.push_back(state.second->robot.pose);
      }
      gui_vars_.estimated_robot_path->SetLastPoseCovariance(
            data_->estimated_poses.rbegin()->second->robot.covariance);
      gui_vars_.estimated_robot_path->ShowCovariance(data_->estimated_poses.size() > 5);
    }

    gui_vars_.data_associations->Clear();

    if (!data_->data_associations->associations.empty()) {
      // update the data associations

      for (auto e : data_->data_associations->associations) {
        RangeFinderObservation& obs = data_->data_associations->observations[e.first];
        LandmarkPtr lm_w = data_->estimated_landmarks.at(e.second);  // should always be present

        // transfer the observation over to the world frame
        Landmark obs_r(obs.observation.range * std::cos(obs.observation.theta),
                       obs.observation.range * std::sin(obs.observation.theta));
        // transfer the observation over to the world frame
        Landmark obs_w = data_->estimated_poses.rbegin()->second->robot * obs_r;

        // finally add to the scene graph
        gui_vars_.data_associations->AddDataAssociation(std::make_pair(obs_w.vec(), lm_w->vec()));

      }

    }
  }
  return true;
}

void Visualizer::RequestFinish() {
  std::unique_lock<std::mutex> lock(status_mutex_);
  finish_requested_ = true;
}

bool Visualizer::CheckFinish() {
  std::unique_lock<std::mutex> lock(status_mutex_);
  return finish_requested_;
}

void Visualizer::SetFinish() {
  std::unique_lock<std::mutex> lock(status_mutex_);
  finished_ = true;
}

bool Visualizer::IsFinished() {
  std::unique_lock<std::mutex> lock(status_mutex_);
  return finished_;
}

bool Visualizer::IsStepping() {
  std::unique_lock<std::mutex> lock(status_mutex_);
  return single_step_;
}

bool Visualizer::SetStepping(bool stepping) {
  std::unique_lock<std::mutex> lock(status_mutex_);
  return single_step_ = stepping;
}

bool Visualizer::IsRunning() {
  std::unique_lock<std::mutex> lock(status_mutex_);
  return running_;
}

void Visualizer::RequestReset() {
  std::unique_lock<std::mutex> lock(status_mutex_);
  reset_requested_ = true;
}

bool Visualizer::IsResetRequested() {
  std::unique_lock<std::mutex> lock(status_mutex_);
  return reset_requested_;
}

const Visualizer::DebugGUIVariables& Visualizer::GetDebugVariables() {
  std::unique_lock<std::mutex> lock(status_mutex_);
  return gui_vars_.ui;
}


void Visualizer::SetReset() {
  std::unique_lock<std::mutex> lock(status_mutex_);
  // reset set on the application, clear the scene graph;
  reset_requested_ = false;
  ResetSceneGraph();

  // also reset the plots
  gui_vars_.log_ptr->Clear();
}


}  // namespace chameleon
