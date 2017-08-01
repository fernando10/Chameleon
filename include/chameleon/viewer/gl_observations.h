// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <SceneGraph/GLObject.h>
#include "chameleon/types.h"
#include "fmt/string.h"
#include "fmt/ostream.h"
#include "glog/logging.h"
#define MAT4_COL_MAJOR_DATA(m) (Eigen::Matrix<float,4,4,Eigen::ColMajor>(m).data())

///
/// \brief The GLObservation class
///Code to render a robots observations at a given timestep
class GLObservations : public SceneGraph::GLObject {
public:
GLObservations() {
  line_color_ << 1.0, 1.0, 0.0, 1.0;
  line_width_ = 1.0;
}
~GLObservations() {}

void SetPoseAndObservations(const chameleon::RobotPose& robot, const chameleon::RangeFinderObservationVector& observations) {
  pose_ = robot;
  observations_ = observations;
}

void SetLineWidth(double line_width) {
  line_width_ = line_width;
}

void SetColor( float R, float G, float B, float A = 1.0) {
  line_color_ << R, G, B, A;
}

void DrawCanonicalObject() override {
  if(observations_.empty()) {
    return;
  }

  glEnable(GL_LINE_SMOOTH);
  glLineWidth(line_width_);
  glColor4f(line_color_[0], line_color_[1], line_color_[2], line_color_[3]);
  Eigen::Vector2d origin_pt = pose_.pose.translation();
  for (const chameleon::RangeFinderObservation& obs : observations_) {
    chameleon::Landmark lm_r(obs.observation.range * std::cos(obs.observation.theta),
                             obs.observation.range * std::sin(obs.observation.theta));
    chameleon::Landmark lm_w = pose_ * lm_r; // transfer landmark from robot frame to world frame
    pangolin::glDrawLine(origin_pt, lm_w.vec());
  }

}

private:
chameleon::RangeFinderObservationVector observations_;
chameleon::RobotPose pose_;
Eigen::Vector4f line_color_;
double line_width_;
};
