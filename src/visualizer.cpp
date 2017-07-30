// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include <thread>
#include <mutex>
#include "fmt/format.h"
#include "summersimulator/viewer/visualizer.h"


/*-----GUI Includes-----------*/
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
/*----------------------------*/

#include <glog/logging.h>

namespace summer
{

Visualizer::Visualizer(const ViewerOptions& options):
  options_(options) {
  viewer_thread_ = std::unique_ptr<std::thread>(new std::thread(&Visualizer::Run, this));
}

void Visualizer::Run() {
  VLOG(1) << "Initializing GUI.";
  // Create a window.
  pangolin::CreateWindowAndBind(options_.window_name,
                                options_.window_width,
                                options_.window_height);
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

  // Reset background color to black.
  glClearColor(0, 0, 0, 1);

  VLOG(1) << "Creating SceneGraph...";

  // Scenegraph to hold GLObjects.
  SceneGraph::GLSceneGraph gl_graph;

  // Attatch a panel on the left side for controls, buttons etc.
  pangolin::CreatePanel("menu").SetBounds(0, 1, 0,
                                          pangolin::Attach::Pix(options_.panel_size));
  pangolin::Var<bool> menu_follow_camera("menu.Follow Camera",true,false);


  // Define Camera Render Object (for view / scene browsing).
  pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(options_.window_width,options_.window_height
                                   ,420, 420, 320, 240, 1E-3, 10*1000),
        pangolin::ModelViewLookAt(5, 0, 100, 0, 0, 0, pangolin::AxisNegZ)
        );

  // Add named OpenGL viewport to window and provide 3D Handler.
  pangolin::View& d_cam = pangolin::CreateDisplay()
                          .SetAspect(-(float)options_.window_width/(float)options_.window_height)
                          .SetHandler(new SceneGraph::HandlerSceneGraph(gl_graph, s_cam))
                          .SetDrawFunction(SceneGraph::ActivateDrawFunctor(gl_graph, s_cam));


  // Create a new view and attach the sub-views to this one.
  pangolin::Display("multi")
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(options_.panel_size), 1.0)
      .SetLayout(pangolin::LayoutEqual)
      .AddDisplay(d_cam);

  SceneGraph::GLGrid gl_grid(150, 1);

  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  // Viewer loop.
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (menu_follow_camera) {
      s_cam.Follow(Twc);
    }

    if (reset_) {
      VLOG(2) << "Reset called (or system start) clearing dispaly and repopulating.";
      gl_graph.Clear();
      // re-popoulate with stuff...
      gl_graph.AddChild(&gl_grid);
      reset_ = false;
    }

    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      if (data_ != nullptr) {
        if(data_->ground_truth_robot_poses != nullptr) {
          for (const auto& robot : *(data_->ground_truth_robot_poses)) {
            DrawRobot(robot);
          }
        }

        if(data_->noisy_robot_poses != nullptr) {
          for (const auto& robot : *(data_->noisy_robot_poses)) {
            DrawRobot(robot, Eigen::Vector3d(1.0, 0.0, 0.0));
          }
        }
      }
    }

    pangolin::FinishFrame();

    // Sleep for a bit.
    usleep(1000);

    if (CheckFinish()) { break; }
  }

  SetFinish();
}

void Visualizer::SetData(ViewerData::Ptr data) {
  std::unique_lock<std::mutex> lock(data_mutex_);
  data_ = data;
}

void Visualizer::DrawRobot(const RobotPose &robot, Eigen::Vector3d color, bool draw_covariance) {
  glColor3f(color[0], color[1], color[2]);
  pangolin::glDrawCirclePerimeter(robot.pose.translation(), 0.3);

  //  Eigen::Vector2d orientation_pt(std::sin(robot.pose.so2().log()) * kRobotRadius,
  //                                 std::cos(robot.pose.so2().log()) * kRobotRadius);
  //  pangolin::glDrawLine(robot.pose.translation(), orientation_pt);
}

void Visualizer::DrawLandmark(const Landmark &landmark, bool draw_covariance) {
  // TODO
}

void Visualizer::RequestFinish() {
  std::unique_lock<std::mutex> lock(finish_mutex_);
  finish_requested_ = true;
}

bool Visualizer::CheckFinish() {
  std::unique_lock<std::mutex> lock(finish_mutex_);
  return finish_requested_;
}

void Visualizer::SetFinish() {
  std::unique_lock<std::mutex> lock(finish_mutex_);
  finished_ = true;
}

bool Visualizer::IsFinished() {
  std::unique_lock<std::mutex> lock(finish_mutex_);
  return finished_;
}

}  // namespace summer
