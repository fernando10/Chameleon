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

  void DrawCanonicalObject() {
    if (landmark_map_.empty()) { return; }

    glLineWidth(1.0f);

    for (const auto& lm : landmark_map_) {
      if (lm.active) {
        glColor4f(map_color_[0], map_color_[1], map_color_[2], map_color_[3]);
      } else {
        glColor4f(0.75f, 0.75f, 0.75f, 1.0f);  // gray for inactive landmarks
      }

      glPushMatrix();

      pangolin::glDrawCirclePerimeter(lm.x(), lm.y(), 0.1);
      pangolin::glDrawCross(lm.x(), lm.y(), 0.1/2.f);

      glPopMatrix();
    }

    //glPushMatrix();
    //glEnable(GL_LINE_SMOOTH);
    //    for(auto& landmark : map_) {
    //      landmark.SetColor(map_color_);
    //      landmark.DrawCanonicalObject();
    //    }
    //glEnd();
    //glPopMatrix();
  }

  //  std::vector<GLLandmark>& GetMapRef() {
  //    return map_;
  //  }

  //  std::vector<std::pair<double, double>>& GetMapRef() {
  //    return pair_map_;
  //  }


  std::vector<chameleon::Landmark>& GetMapRef() {
    return landmark_map_;
  }


  void Clear() {
    //map_.clear();
    landmark_map_.clear();
  }

  void SetColor( float R, float G, float B, float A = 1.0) {
    map_color_ << R, G, B, A;
  }

private:
//  std::vector<GLLandmark> map_;
//  std::vector<std::pair<double, double>> pair_map_;
  std::vector<chameleon::Landmark> landmark_map_;
  Eigen::Vector4f map_color_;
};
