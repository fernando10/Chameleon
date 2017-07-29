// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <thread>
#include <mutex>
#include "summersimulator/types.h"
#include "summersimulator/util.h"

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
    RobotPoseVectorPtr robot_poses;
  };

  Visualizer(const ViewerOptions& options);

  void SetData(ViewerData::Ptr data);

  void RequestFinish();
  bool IsFinished();

private:
  const double kRobotRadius = 0.3;

  void Run();
  void SetFinish();
  bool CheckFinish();

  void DrawRobot(const RobotPose& robot,  bool draw_covariance = false);
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

}  // namespace elninho
