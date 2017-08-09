// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <Eigen/Eigen>
#include "chameleon/types.h"
#include "chameleon/viewer/gl_landmark.h"
#include "SceneGraph/GLText.h"
#include <Eigen/Eigenvalues>

///
/// \brief The GLMap class
/// Code to render a set of landmarks
class GLMap : public SceneGraph::GLObject
{
public:
  GLMap() {
    map_color_ << 0.f, 1.f, 0.f, 1.f;
    draw_persistence_labels_ = false;
    draw_variance_ = false;
  }

  void DrawCanonicalObject() {
    if (landmark_vec_.empty()) { return; }

    glLineWidth(1.0f);
    Eigen::Vector4f color;

    for (const auto& lm : landmark_vec_) {
      if (lm.active) {
        color << map_color_[0], map_color_[1], map_color_[2], map_color_[3];
      } else {
        double prob = lm.persistence_prob;
        color << 1.f - 0.25f*float(prob), float(prob), float(prob), 1.0f;
      }

      glColor4f(color[0], color[1], color[2], color[3]);

      glPushMatrix();

      pangolin::glDrawCirclePerimeter(lm.x(), lm.y(), 0.1);
      pangolin::glDrawCross(lm.x(), lm.y(), 0.1/2.f);

      glPopMatrix();

      if (draw_persistence_labels_) {
        SceneGraph::GLText text(std::to_string(lm.persistence_prob), lm.x(), lm.y(), -0.3);
        text.SetPosition(lm.x(), lm.y(), -0.3);
        text.DrawObjectAndChildren();
      }


    }
  }

  void Plot2dErrorElipse(double chisquare, Eigen::Vector2d mean, const Eigen::Matrix2d&cov) {
//    Eigen::EigenSolver<Eigen::Matrix2d> solver(cov);
//    // calc angle between the largest eigenvector and the x axis
//    double angle  = std::atan2(solver.eigenvectors().col(0)[1], solver.eigenvectors().col(0)[0]);

//    // size of major and minor axes
//    double half_major_axis_size = chisquare * std::sqrt(solver.eigenvalues()[0]);
//    double half_minor_axis_size = chisquare * std::sqrt(solver.eigenvalues()[1]);

  }

  std::vector<chameleon::Landmark>& GetMapRef() {
    return landmark_vec_;
  }

  std::vector<SceneGraph::GLText>& GetMapLabelsRef() {
    return landmark_labels_vec_;
  }

  void Clear() {
    landmark_vec_.clear();
  }

  void SetColor( float R, float G, float B, float A = 1.0) {
    map_color_ << R, G, B, A;
  }

  void SetShowVariance(bool plot_variance) {
    draw_variance_ = plot_variance;
  }

  void SetShowPersistenceLabels(bool show) {
    draw_persistence_labels_ = show;
  }

private:
  std::vector<chameleon::Landmark> landmark_vec_;
  std::vector<SceneGraph::GLText> landmark_labels_vec_;
  Eigen::Vector4f map_color_;
  bool draw_persistence_labels_;
  bool draw_variance_;
};
