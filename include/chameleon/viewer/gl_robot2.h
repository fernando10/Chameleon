// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <Eigen/Eigen>
#include <sophus/se2.hpp>

#include <SceneGraph/GLObject.h>

///
/// \brief The GLRobot2 class
/// Code to render a robot
class GLRobot2 : public SceneGraph::GLObject {
public:
  GLRobot2() {
    robot_color_ << 1.0, 1.0, 0.0, 1.0;
    draw_covariance_ = false;
  }

  void DrawCanonicalObject() override {
    glLineWidth(2.0f);
    glColor4f(robot_color_[0], robot_color_[1], robot_color_[2], robot_color_[3]);
    pangolin::glDrawCirclePerimeter(0, 0, robot_radius_);
    if (draw_covariance_) {
      glDraw2dCovariance(covariance_, 9);
    }
    //pangolin::glDrawLine(0, 0, robot_radius_, 0);  // orientation
  }

  void SetColor( float R, float G, float B, float A = 1.0) {
    robot_color_ << R, G, B, A;
  }


  inline void glDraw2dCovariance( Eigen::Matrix2d C, double k )
  {

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(0.5 * (C + C.transpose()));
    Eigen::Matrix2d eigenvalues = es.eigenvalues().asDiagonal();
    Eigen::Matrix2d eigenvectors = es.eigenvectors();
    Eigen::Matrix2d A = eigenvectors * (eigenvalues * k).sqrt();

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

  void SetCovaraince(const Eigen::Matrix2d& cov) {
    covariance_ = cov;
  }

  void SetDrawCovariance(bool draw) {
    draw_covariance_ = draw;
  }

private:
  Eigen::Vector4f robot_color_ = Eigen::Vector4f::Zero();
  Eigen::Matrix2d covariance_;
  bool draw_covariance_;
  float robot_radius_ = 0.1f;

};
