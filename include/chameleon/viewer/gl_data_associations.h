// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <SceneGraph/GLObject.h>
#include "chameleon/types.h"
#include "fmt/string.h"
#include "fmt/ostream.h"
#include "glog/logging.h"

///
/// \brief The GLDataAssociation class
///Code to render the data associations between measurements and map features
class GLDataAssociations : public SceneGraph::GLObject {
public:
GLDataAssociations() {
  line_color_ << 1.0, 1.0, 0.0, 1.0;
  line_width_ = 2.0;
}
~GLDataAssociations() {}

void SetLineWidth(double line_width) {
  line_width_ = line_width;
}

void SetColor( float R, float G, float B, float A = 1.0) {
  line_color_ << R, G, B, A;
}

void Clear() {
  data_associations_.clear();
}

void AddDataAssociation(std::pair<Eigen::Vector2d, Eigen::Vector2d> association) {
  data_associations_.push_back(association);
}

void DrawCanonicalObject() override {
  if(data_associations_.empty()) {
    return;
  }

  glEnable(GL_LINE_SMOOTH);
  glLineWidth(line_width_);
  glColor4f(line_color_[0], line_color_[1], line_color_[2], line_color_[3]);

  for (auto it = data_associations_.begin(); it != data_associations_.end(); ++it) {
    pangolin::glDrawLine(it->first, it->second);
  }

}

private:
typedef std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> DataAssociations;
DataAssociations data_associations_;
Eigen::Vector4f line_color_;
double line_width_;
};
