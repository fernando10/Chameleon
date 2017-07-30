// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <thread>
#include <mutex>
#include "summersimulator/types.h"
#include "summersimulator/util.h"
#include "summersimulator/viewer/gl_landmark.h"
#include "summersimulator/viewer/gl_robot.h"

#include "fmt/format.h"

/*-----GUI Includes-----------*/
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/GLDynamicGrid.h>
/*----------------------------*/

namespace summer
{

class Visualizer {
public:
  struct ViewerOptions {
  public:
    std::string window_name = "Viewer";
    int window_height = 768;
    int window_width = 1024;
    unsigned int panel_size = 180;
  };

  struct ViewerData {
    typedef std::shared_ptr<ViewerData> Ptr;
    RobotPoseVectorPtr ground_truth_robot_poses;
    RobotPoseVectorPtr noisy_robot_poses;
  };

  struct GuiVars {
    pangolin::OpenGlRenderState camera2d;
    pangolin::OpenGlRenderState camera3d;  // currently unused...for projective views
    SceneGraph::GLSceneGraph scene_graph;  // Scene Graph to hold GLObjects and realtive transforms
    SceneGraph::GLDynamicGrid dynamic_grid;  // Grid object to be the world plane
    std::unique_ptr<SceneGraph::HandlerSceneGraph> handler;
    SceneGraph::AxisAlignedBoundingBox aa_bounding_box;
    std::unique_ptr<pangolin::View> world_view_ptr;
    std::unique_ptr<pangolin::View> panel_view_ptr;
    std::unique_ptr<pangolin::View> multi_view_ptr;
    SceneGraph::GLLight light;
  };

  Visualizer(const ViewerOptions& options);

  void SetData(ViewerData::Ptr data);

  void RequestFinish();
  bool IsFinished();

private:
  void InitGui();
  void Run();
  void SetFinish();
  bool CheckFinish();

  void DrawRobot(const RobotPose& robot, Eigen::Vector3d color = Eigen::Vector3d(0., 0., 1.),
                 bool draw_covariance = false);
  void DrawLandmark(const Landmark& landmark, bool draw_covariance = false);

  const ViewerOptions& options_;
  GuiVars gui_vars_;
  ViewerData::Ptr data_;
  std::unique_ptr<std::thread> viewer_thread_;
  std::mutex finish_mutex_;
  std::mutex data_mutex_;
  bool finished_ = false;
  bool finish_requested_ = false;
  bool reset_ = true;
};

}  // namespace summer
