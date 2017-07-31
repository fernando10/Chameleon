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

void Visualizer::InitGui() {
  VLOG(1) << "Initializing GUI.";
  // Create a window.
  pangolin::CreateWindowAndBind(options_.window_name,
                                options_.window_width,
                                options_.window_height);

  // setup camera
  gui_vars_.camera.SetModelViewMatrix(pangolin::ModelViewLookAt(3, 3, 4, 0, 0, 0, pangolin::AxisZ));
  gui_vars_.camera.SetProjectionMatrix(pangolin::ProjectionMatrix(options_.window_width, options_.window_height,
                                                                  420, 420, options_.window_width/2., options_.window_height/2. , 0.01, 5000));

  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  // Reset background color to black.
  glClearColor(0, 0, 0, 1);

  // create scene graph objects
  gui_vars_.light = SceneGraph::GLLight(10, 10, -1000);
  gui_vars_.scene_graph.AddChild(&gui_vars_.light);

  gui_vars_.scene_graph.AddChild(&gui_vars_.dynamic_grid);

  // add robot path to the scene graph (gt and noisy)
  gui_vars_.scene_graph.AddChild(&gui_vars_.gt_robot_path);
  gui_vars_.gt_robot_path.SetColor(0, 0, 1);
  gui_vars_.scene_graph.AddChild(&gui_vars_.noisy_robot_path);
  gui_vars_.noisy_robot_path.SetColor(1, 0, 0);

  // and the ground truth map
  gui_vars_.scene_graph.AddChild(&gui_vars_.ground_truth_map);

  // create view that will contain the robot/landmarks
  gui_vars_.world_view_ptr.reset(&pangolin::CreateDisplay()
                                 .SetAspect(-(float)options_.window_width/(float)options_.window_height)
                                 .SetBounds(0, 1.0, 0, 1.0));

  // create view for the controls panel
  gui_vars_.panel_view_ptr.reset(&pangolin::CreatePanel("ui").
                                 SetBounds(0, 1.0, 0, pangolin::Attach::Pix(options_.panel_size)));

  // create handler to allow user input to update modelview matrix and flow through to the scene graph
  gui_vars_.handler.reset(new SceneGraph::HandlerSceneGraph(gui_vars_.scene_graph, gui_vars_.camera,
                                                            pangolin::AxisNegZ));

  gui_vars_.world_view_ptr->SetHandler(gui_vars_.handler.get());
  gui_vars_.world_view_ptr->SetDrawFunction(SceneGraph::ActivateDrawFunctor(gui_vars_.scene_graph,
                                                                            gui_vars_.camera));

  // create a container view in case we want to have multiple views on the rigth side (maybe add some plots, etc)
  gui_vars_.multi_view_ptr.reset(&pangolin::Display("multi")
                                 .SetBounds(0.0, 1.0, pangolin::Attach::Pix(options_.panel_size), 1.0)
                                 .SetLayout(pangolin::LayoutEqualVertical));

  // for now just add the world view
  gui_vars_.multi_view_ptr->AddDisplay(*(gui_vars_.world_view_ptr));


  // set the keybindings
  pangolin::RegisterKeyPressCallback('p', std::bind(&Visualizer::SwitchProjection,
                                                    this, ProjectionMatrixTypes::Perspective));
  pangolin::RegisterKeyPressCallback('o', std::bind(&Visualizer::SwitchProjection,
                                                    this, ProjectionMatrixTypes::Orthographic));
  pangolin::RegisterKeyPressCallback(' ' , [&]() { running_ = !running_; });
  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT,
                                     [&]() { single_step_ = true; });
}

void Visualizer::Run() {

  VLOG(1) << "Running viewer thread...";
  InitGui();  // the gui needs to be initialized in the same thread

  // Viewer loop.
  while (!pangolin::ShouldQuit() || CheckFinish()) {

    // clear whole screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    pangolin::FinishFrame();

    // Pause for 1/60th of a second
    std::this_thread::sleep_for(std::chrono::microseconds(1000 / 60));
  }
  SetFinish();
}

void Visualizer::SetData(ViewerData::Ptr data) {
  std::unique_lock<std::mutex> lock(data_mutex_);
  data_ = data;

  // load the gt landmarks now since these shouldn't change
  if (data_->ground_truth_map != nullptr) {
    for (const auto& lm : *data_->ground_truth_map) {
      gui_vars_.ground_truth_map.GetMapRef().push_back(GLLandmark(lm));
    }
  }
}

bool Visualizer::AddTimesteps(std::vector<size_t> timesteps) {
  std::unique_lock<std::mutex>(data_mutex_);

  for (const size_t& ts : timesteps) {
    if (data_ == nullptr) {
      LOG(ERROR) << fmt::format("Requested to add timestep: {} but viewer has no valid data pointer...", ts);
      return false;
    }
    // add the robot pose at the current time
    if (data_->ground_truth_robot_poses != nullptr && data_->ground_truth_robot_poses->size() > ts) {
      // data exsits, add this pose to the display
      RobotPose& robot = data_->ground_truth_robot_poses->at(ts);
      std::vector<Sophus::SE2d>& poses_path_ref = gui_vars_.gt_robot_path.GetPathRef();
      poses_path_ref.push_back(robot.pose);
      VLOG(3) << fmt::format("Added pose to path at: {}, {}", robot.pose.translation().x(), robot.pose.translation().y());
    } else {
      LOG(ERROR) << fmt::format("Error adding robot pose at timestep: {}, either data is null or index does not exist.", ts);
    }

    // lets also add the noisy robot poses at the current time
    if (data_->noisy_robot_poses != nullptr && data_->noisy_robot_poses->size() > ts) {
      // data exsits, add this pose to the display
      RobotPose& noisy_robot = data_->noisy_robot_poses->at(ts);
      std::vector<Sophus::SE2d>& poses_path_ref = gui_vars_.noisy_robot_path.GetPathRef();
      poses_path_ref.push_back(noisy_robot.pose);
      VLOG(3) << fmt::format("Added noisy pose to path at: {}, {}", noisy_robot.pose.translation().x(), noisy_robot.pose.translation().y());
    } else {
      LOG(ERROR) << fmt::format("Error adding noisy robot pose at timestep: {}, either data is null or index does not exist.", ts);
    }

    // add the ground truth landmark observations
    if (data->ground_truth_observation_map.size() > ts) {
      // get the observations for this timestep
      const RangeFinderObservationVector& gt_observations = data->ground_truth_observation_map.at(ts);
      for (const RangeFinderObservation& obs : gt_observations) {
        // TODO
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


}  // namespace chameleon
