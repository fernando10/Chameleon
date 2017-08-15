// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <Eigen/Eigen>
#include "chameleon/types.h"

#include <SceneGraph/GLObject.h>

class GLLine : public SceneGraph::GLObject
{
public:
  GLLine() {
    color_ << 0.0, 1.0, 1.0, 1.0;
  }

  void DrawCanonicalObject() {
    if(pts.size() < 2) {
      return;
    }
    glColor4f(color_[0], color_[1], color_[2], color_[3]);
    glLineWidth(1.0f);
    glPushMatrix();
    pangolin::glDrawLine(pts[0], pts[1]);
    glPopMatrix();
  }

  void SetColor( float R, float G, float B, float A = 1.0) {
    color_ << R, G, B, A;
  }

  void SetPoints(Eigen::Vector2d start, Eigen::Vector2d end) {
    pts.clear();
    pts.push_back(start);
    pts.push_back(end);
  }

private:
  Eigen::Vector4f color_ = Eigen::Vector4f::Zero();
  std::vector<Eigen::Vector2d> pts;

};
