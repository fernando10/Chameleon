// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <Eigen/Eigen>
#include "gl_robot.h"
#include <sophus/se2.hpp>
#include "fmt/string.h"
#include "fmt/ostream.h"

#include <SceneGraph/GLObject.h>
#define MAT4_COL_MAJOR_DATA(m) (Eigen::Matrix<float,4,4,Eigen::ColMajor>(m).data())

///
/// \brief The GLPathAbs class
///Code to render robot trajectory as a line
class GLPathAbs : public SceneGraph::GLObject {
public:
  GLPathAbs() {
    line_color_ << 1.0, 1.0, 0.0, 1.0;
  }

  ~GLPathAbs() {}

  void DrawCanonicalObject() override {
    if (path_.empty()) {
      return;
    }

    Eigen::Matrix3f pose_mat;

    glPushMatrix();
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(2.0);
    glColor4f(line_color_[0], line_color_[1], line_color_[2], line_color_[3]);
    glBegin(GL_LINE_STRIP);
    for (const auto& pose : path_) {
      pose_mat = pose.matrix().cast<float>();
      glVertex3f(pose_mat(0, 2), pose_mat(1, 2), 0.f);
    }
    glEnd();
    glPopMatrix();

    // draw the robot at the end
    robot_.SetColor(line_color_[0], line_color_[1], line_color_[2], line_color_[3]);
    robot_.SetPose(path_.back());
    robot_.DrawCanonicalObject();
  }

  void SetColor(Eigen::Vector4f color) {
    line_color_ = color;
  }

  void Clear() {
    path_.clear();
  }

  void SetColor(float R, float G, float B, float A = 1.f) {
    line_color_[0] = R;
    line_color_[1] = G;
    line_color_[2] = B;
    line_color_[3] = A;
  }

  std::vector<Sophus::SE2d>& GetPathRef() {
    return path_;
  }

  void SetLastPoseCovariance(Eigen::Matrix3d cov) {
    robot_.SetCovariance(cov);
  }

  void ShowCovariance(bool show) {
    robot_.ShowCovariance(show);
  }

private:
  Eigen::Vector4f line_color_;
  GLRobot robot_;
  std::vector<Sophus::SE2d> path_;
};
