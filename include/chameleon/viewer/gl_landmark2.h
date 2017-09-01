// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include "chameleon/types.h"

#include <SceneGraph/GLObject.h>
///
/// \brief The GLLandmark class
/// Code to render a landmark
class GLLandmark2 : public SceneGraph::GLObject
{
public:
  GLLandmark2() {
    lm_color_ << 0.0, 1.0, 0.0, 1.0;
    draw_covariance_ = false;
    confidence_ = 7.0131157;
    covariance_.setIdentity();
  }
  ~GLLandmark2() {}

  void DrawCanonicalObject() {

    glColor4f(lm_color_[0], lm_color_[1], lm_color_[2], lm_color_[3]);
    pangolin::glDrawCirclePerimeter(0, 0, lm_radius_);
    pangolin::glDrawCross(0, 0, lm_radius_/2.f);

    if (draw_covariance_) {
      glColor4f(1.f, 0.f, 1.f, 1.f);  // magenta

      glDraw2dCovariance(covariance_);
    }

  }

  inline void glDraw2dCovariance( Eigen::Matrix2d C )
  {

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(0.5 * (C + C.transpose()));
    Eigen::Matrix2d eigenvalues = es.eigenvalues().asDiagonal();
    Eigen::Matrix2d eigenvectors = es.eigenvectors();
    Eigen::Matrix2d A = eigenvectors * (eigenvalues * confidence_).sqrt();

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

  void SetCovariancePercentile(double conf) {
    confidence_ = conf;
  }



private:
  Eigen::Vector4f lm_color_ = Eigen::Vector4f::Zero();
  Eigen::Matrix2d covariance_;
  double confidence_;
  bool draw_covariance_;
  float lm_radius_ = 0.1f;
};
