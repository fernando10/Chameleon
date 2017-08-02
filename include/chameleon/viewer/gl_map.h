// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <Eigen/Eigen>
#include "chameleon/types.h"
#include "chameleon/viewer/gl_landmark.h"

///
/// \brief The GLMap class
/// Code to render a set of landmarks
class GLMap : public SceneGraph::GLObject
{
public:
  GLMap() {
    map_color_ << 0.f, 1.f, 0.f, 1.f;
  }
  ~GLMap() {}

  void DrawCanonicalObject() {
    if (map_.empty()) { return; }
    glLineWidth(1.0f);
    glPushMatrix();
    glEnable(GL_LINE_SMOOTH);
    for(auto& landmark : map_) {
      landmark.SetColor(map_color_);
      landmark.DrawCanonicalObject();
    }
    glEnd();
    glPopMatrix();
  }

  std::vector<GLLandmark>& GetMapRef() {
    return map_;
  }

  void Clear() {
    map_.clear();
  }

  void SetColor( float R, float G, float B, float A = 1.0) {
    map_color_ << R, G, B, A;
  }

private:
  std::vector<GLLandmark> map_;
  Eigen::Vector4f map_color_;
};
