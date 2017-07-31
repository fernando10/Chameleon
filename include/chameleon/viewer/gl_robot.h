// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <Eigen/Eigen>
#include <sophus/se2.hpp>

#include <SceneGraph/GLObject.h>
#define MAT4_COL_MAJOR_DATA(m) (Eigen::Matrix<float,4,4,Eigen::ColMajor>(m).data())

///
/// \brief The GLRobot class
/// Code to render a robot
class GLRobot : public SceneGraph::GLObject {
 public:
  GLRobot() {
    robot_color_ << 1.0, 1.0, 0.0, 1.0;
  }

  GLRobot(const Sophus::SE2d& pose): GLRobot() {
    robot_pose_ = pose;
    ConvertPose2d3d();
  }

  void SetPose(const Sophus::SE2d& pose) {
    robot_pose_ = pose;
    ConvertPose2d3d();
  }

  void ConvertPose2d3d() {
    robot_pose_3d_.setIdentity();
    robot_pose_3d_.topLeftCorner<2, 2>() = robot_pose_.so2().matrix().cast<float>();
    robot_pose_3d_.block<2, 1>(0, 3) = robot_pose_.translation().cast<float>();
  }

  ~GLRobot() {
  }

  void DrawCanonicalObject() override {

    glLineWidth(2.0f);

    if (draw_robot_) {
      glPushMatrix();

      glMultMatrixf(MAT4_COL_MAJOR_DATA(robot_pose_3d_));
      glColor4f(robot_color_[0], robot_color_[1], robot_color_[2], robot_color_[3]);
      pangolin::glDrawCirclePerimeter(0, 0, robot_radius_);
      pangolin::glDrawLine(0, 0, robot_radius_, 0);  // orientation

      glPopMatrix();
    }

  }

  void SetColor( float R, float G, float B, float A = 1.0) {
    robot_color_ << R, G, B, A;
  }

private:
  Eigen::Vector4f robot_color_ = Eigen::Vector4f::Zero();
  Sophus::SE2d robot_pose_;
  Eigen::Matrix4f robot_pose_3d_ = Eigen::Matrix4f::Identity();
  bool draw_robot_ = true;
  float robot_radius_ = 0.3f;

};
