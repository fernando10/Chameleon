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
  GLMap() {}
  ~GLMap() {}

  void DrawCanonicalObject() {
    if (map_.empty()) { return; }
    glLineWidth(1.0f);
    glPushMatrix();
    glEnable(GL_LINE_SMOOTH);
    for(auto& landmark : map_) {
      landmark.DrawCanonicalObject();
    }
    glEnd();
    glPopMatrix();
  }

  std::vector<GLLandmark>& GetMapRef() {
    return map_;
  }

private:
  std::vector<GLLandmark> map_;
};
