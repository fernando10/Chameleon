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
class GLLandmark : public SceneGraph::GLObject
{
 public:
  GLLandmark() {
    lm_color_ << 0.0, 1.0, 0.0, 1.0;
  }

  GLLandmark(const Eigen::Vector2d& position): GLLandmark() {
    lm_position_ = position;
  }

  GLLandmark(const chameleon::Landmark& lm): GLLandmark() {
    lm_position_[0] = lm.x();
    lm_position_[1] = lm.y();
  }

  ~GLLandmark() {}

  void DrawCanonicalObject() {

    if (draw_lm_) {
      glPushMatrix();

      glColor4f(lm_color_[0], lm_color_[1], lm_color_[2], lm_color_[3]);
      pangolin::glDrawCirclePerimeter(lm_position_.x(), lm_position_.y(), lm_radius_);
      pangolin::glDrawCross(lm_position_.x(), lm_position_.y(), lm_radius_/2.f);

      glPopMatrix();
    }
  }

  void SetColor( float R, float G, float B, float A = 1.0) {
    lm_color_ << R, G, B, A;
  }

  void SetColor( Eigen::Vector4f color) {
    lm_color_ = color;
  }

private:
  Eigen::Vector4f lm_color_ = Eigen::Vector4f::Zero();
  Eigen::Vector2d lm_position_;
  bool draw_lm_ = true;
  float lm_radius_ = 0.1f;
};
