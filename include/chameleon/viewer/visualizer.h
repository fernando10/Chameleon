// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <thread>
#include <mutex>
#include "chameleon/types.h"
#include "chameleon/util.h"
#include "chameleon/viewer/gl_landmark.h"
#include "chameleon/viewer/gl_robot.h"

#include "fmt/format.h"

/*-----GUI Includes-----------*/
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/GLDynamicGrid.h>
/*----------------------------*/

namespace chameleon
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

  // Struct for passing data from the application to the viewer
  struct ViewerData {
    ViewerData() {
      ground_truth_robot_poses = std::make_shared<RobotPoseVector>();
      noisy_robot_poses = std::make_shared<RobotPoseVector>();
      ground_truth_map = std::make_shared<LandmarkVector>();
    }

    typedef std::shared_ptr<ViewerData> Ptr;
    RobotPoseVectorPtr ground_truth_robot_poses;
    RobotPoseVectorPtr noisy_robot_poses;
    LandmarkVectorPtr ground_truth_map;
  };

  struct GuiVars {
    pangolin::OpenGlRenderState camera;
    pangolin::OpenGlRenderState camera3d;
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

  enum class ProjectionMatrixTypes {
    Perspective,
    Orthographic
  };

  void InitGui();
  void SwitchProjection(ProjectionMatrixTypes type);
  void Run();
  void SetFinish();
  bool CheckFinish();

   // TEMP
  std::vector<std::unique_ptr<GLRobot>> robots_to_draw;
  std::vector<std::unique_ptr<GLLandmark>> lm_to_draw;


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

}  // namespace chameleon
