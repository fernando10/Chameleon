// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <thread>

#include "types.h"
#include "util.h"

namespace elninho
{

struct ViewerOptions
{
public:
  std::string window_name = "Viewer";
  int window_height = 768;
  int window_width = 1024;
  unsigned int panel_size = 180;
};

class Visualizer {
public:
  Visualizer(const ViewerOptions& options);

  void RequestFinish();
  bool IsFinished();

private:
  void Run();
  void SetFinish();
  bool CheckFinish();

  void DrawRobot(const RobotPose& pose, bool draw_covariance = false);
  void DrawLandmark(const Landmark& landmark, bool draw_covariance = false);

  const ViewerOptions& options_;
  std::unique_ptr<std::thread> viewer_thread_;
  std::mutex finish_mutex_;
  bool finished_ = false;
  bool finish_requested_ = false;
  std::mutex mutex_finish_;
  bool reset_ = true;
};

}  // namespace elninho
