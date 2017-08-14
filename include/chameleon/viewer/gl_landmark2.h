// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <Eigen/Eigen>
#include "chameleon/types.h"

#include <SceneGraph/GLObject.h>
#define MAT4_COL_MAJOR_DATA(m) (Eigen::Matrix<float,4,4,Eigen::ColMajor>(m).data())

///
/// \brief The GLLandmark class
/// Code to render a landmark
class GLLandmark2 : public SceneGraph::GLObject
{
public:
  GLLandmark2() {
    lm_color_ << 0.0, 1.0, 0.0, 1.0;
    draw_covariance_ = false;
    covariance_.setIdentity();
  }
  ~GLLandmark2() {}

  void DrawCanonicalObject() {

    glColor4f(lm_color_[0], lm_color_[1], lm_color_[2], lm_color_[3]);
    pangolin::glDrawCirclePerimeter(0, 0, lm_radius_);
    pangolin::glDrawCross(0, 0, lm_radius_/2.f);

    if (draw_covariance_) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(0.5 * (covariance_  + covariance_.transpose()));
      Eigen::Vector2d eigenvalues = es.eigenvalues();
      Eigen::Matrix2d eigenvectors = es.eigenvectors();

      // compute angle between largest eigenvector and the x-axis
      double angle = std::atan2(eigenvectors(1, 1), eigenvectors(1, 0));

      // compute size of major and minor axes
      double half_major_axis = chi_squared_threshold_ * std::sqrt(eigenvalues(1));  // largest eigenvalue
      double half_minor_axis = chi_squared_threshold_ * std::sqrt(eigenvalues(0));  // smallest eigenvalue

      glDrawEllipsePerimeter(half_major_axis, half_minor_axis);
    }

  }

  inline void glDrawEllipsePerimeter( float rx, float ry )
  {
      const int N = 50;
      GLfloat verts[N*2];

      const float TAU_DIV_N = 2*(float)M_PI/N;
      for(int i = 0; i < N*2; i+=2) {
          verts[i] =   rx * cos(i*TAU_DIV_N);
          verts[i+1] = ry * sin(i*TAU_DIV_N);
      }

      pangolin::glDrawVertices<float>(N, verts, GL_LINES, 2);
  }

  void SetColor( float R, float G, float B, float A = 1.0) {
    lm_color_ << R, G, B, A;
  }

  void SetColor( Eigen::Vector4f color) {
    lm_color_ = color;
  }

  void SetCovariance(Eigen::Matrix2d cov) {
    covariance_ = cov;
  }

  void DrawCovariance(bool draw_cov) {
    draw_covariance_ = draw_cov;
  }

  void SetChiSquaredThreshold(double thresh) {
    chi_squared_threshold_ = thresh;
  }



private:
  Eigen::Vector4f lm_color_ = Eigen::Vector4f::Zero();
  Eigen::Matrix2d covariance_;
  double chi_squared_threshold_ = 0.0506;
  bool draw_covariance_;
  float lm_radius_ = 0.1f;
};
