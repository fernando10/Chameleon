// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <thread>
#include <mutex>
#include "summersimulator/types.h"
#include "summersimulator/util.h"
#include "summersimulator/viewer/gl_landmark.h"
#include "summersimulator/viewer/gl_robot.h"

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

  Visualizer(const ViewerOptions& options);

  void SetData(ViewerData::Ptr data);

  void RequestFinish();
  bool IsFinished();

private:
  void Run();
  void SetFinish();
  bool CheckFinish();

  void DrawRobot(const RobotPose& robot, Eigen::Vector3d color = Eigen::Vector3d(0., 0., 1.),
                 bool draw_covariance = false);
  void DrawLandmark(const Landmark& landmark, bool draw_covariance = false);

  const ViewerOptions& options_;
  ViewerData::Ptr data_;
  std::unique_ptr<std::thread> viewer_thread_;
  std::mutex finish_mutex_;
  std::mutex data_mutex_;
  bool finished_ = false;
  bool finish_requested_ = false;
  bool reset_ = true;
};

}  // namespace summer
