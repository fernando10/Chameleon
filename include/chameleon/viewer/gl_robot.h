// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <sophus/se2.hpp>
#include "chameleon/types.h"

#include <SceneGraph/GLObject.h>
#define MAT4_COL_MAJOR_DATA(m) (Eigen::Matrix<float,4,4,Eigen::ColMajor>(m).data())

///
/// \brief The GLRobot class
/// Code to render a robot
class GLRobot : public SceneGraph::GLObject {
 public:
  GLRobot() {
    robot_color_ << 1.0, 1.0, 0.0, 1.0;
    covariance_ = Eigen::Matrix3d::Identity();
    show_covariance_ = false;
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

      if (show_covariance_) {
        glDraw2dCovariance(covariance_.block<2, 2>(chameleon::RobotPose::kIndexTransCov,
                                                   chameleon::RobotPose::kIndexTransCov), 9);  // only x, y covariance
      }

      glPopMatrix();
    }

  }

  inline void glDraw2dCovariance(Eigen::Matrix2d C, double conf )
  {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(0.5 * (C + C.transpose()));
    Eigen::Matrix2d eigenvalues = es.eigenvalues().asDiagonal();
    Eigen::Matrix2d eigenvectors = es.eigenvectors();
    Eigen::Matrix2d A = eigenvectors * (eigenvalues * conf).sqrt();

      const int N = 100;  // number of vertices
      Eigen::MatrixXd pts(N, 2);
      double inc = 2 * M_PI / N;
      for(size_t i = 0; i < N; ++i) {
        pts.row(i) = Eigen::Vector2d(std::cos(i * inc), std::sin(i * inc));
      }
      pts = pts * A;

      GLfloat verts[N*2];
      for(size_t i = 0; i < N; ++i ){
        verts[i*2] = (float)pts.row(i)[0];
        verts[i*2+1] = (float)pts.row(i)[1];
      }
      //pangolin::glDrawVertices<float>(N, verts, GL_LINE_LOOP, 2);
      glVertexPointer(2, GL_FLOAT, 0, verts);
      glEnableClientState(GL_VERTEX_ARRAY);
      glDrawArrays(GL_LINE_LOOP, 0, N);
      glDisableClientState(GL_VERTEX_ARRAY);
  }

  void SetColor( float R, float G, float B, float A = 1.0) {
    robot_color_ << R, G, B, A;
  }

  void SetCovariance(Eigen::Matrix3d cov) {
    covariance_ =  cov;
  }

  void ShowCovariance(bool show) {
    show_covariance_ = show;
  }

private:
  Eigen::Vector4f robot_color_ = Eigen::Vector4f::Zero();
  Sophus::SE2d robot_pose_;
  bool show_covariance_;
  Eigen::Matrix3d covariance_;
  Eigen::Matrix4f robot_pose_3d_ = Eigen::Matrix4f::Identity();
  bool draw_robot_ = true;
  float robot_radius_ = 0.3f;

};
