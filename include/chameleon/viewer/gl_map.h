// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
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
    color_based_on_persistence_ = false;
    draw_variance_ = false;
    draw_landmark_id_ = false;
    draw_persistence_weights_ = false;
    selected_lm_id_ = 1e10;
  }

  void DrawCanonicalObject() {
    if (landmark_vec_.empty()) { return; }

    glLineWidth(1.0f);
    Eigen::Vector4f color;

    for (const auto& lm : landmark_vec_) {
      if (lm.active) {
        color << map_color_[0], map_color_[1], map_color_[2], map_color_[3];
      } else {
        color << 1, 1, 1, 1;

      }
      if (color_based_on_persistence_) {
        double prob = lm.persistence_prob;
        color << 1.f - 0.25f*float(prob), float(prob), float(prob), 1.0f;
      }

      glColor4f(color[0], color[1], color[2], color[3]);

      glPushMatrix();

      pangolin::glDrawCirclePerimeter(lm.x(), lm.y(), 0.1);
      pangolin::glDrawCross(lm.x(), lm.y(), 0.1/2.f);

      if (draw_variance_) {
        glDraw2dCovariance(lm.x(), lm.y(), lm.covariance, 9);
      }

      if (lm.id == selected_lm_id_) {
        glColor4f(1.0, 1.0, 0.0, 1.0);
        double side = 1;
        pangolin::glDrawRectPerimeter(lm.x() + side/2, lm.y() + side/2, lm.x() - side/2, lm.y() - side/2);
      }


      glPopMatrix();

      if (draw_persistence_labels_) {
        SceneGraph::GLText text(std::to_string(lm.persistence_prob), lm.x(), lm.y(), -0.3);
        text.SetPosition(lm.x(), lm.y(), -0.3);
        text.DrawObjectAndChildren();
      }else if(draw_landmark_id_) {
        SceneGraph::GLText text(std::to_string(lm.id), lm.x(), lm.y(), -0.3);
        text.SetPosition(lm.x(), lm.y(), -0.3);
        text.DrawObjectAndChildren();
      }
    }

    std::map<uint64_t, size_t> lm_id_2_idx_map;

    size_t idx = 0;
    for (const auto& lm : landmark_vec_) {
      lm_id_2_idx_map[lm.id] = idx;
      idx++;
    }


    if (draw_persistence_weights_) {
      //std::cerr << "drawing persistence weights.." << std::endl;
      if (persistence_weights_ != nullptr) {

        size_t lm_idx = 0;
        for (const auto& e1 : *(persistence_weights_)) {
          glColor4f(lm_idx % 2, lm_idx % 2, lm_idx % 3, 1.0);

          uint64_t lm_id = e1.first;
          Eigen::Vector2d root_position = landmark_vec_[lm_id_2_idx_map.at(lm_id)].vec();

          for (const auto& e2 : e1.second) {

            if(e2.first == lm_id) {
              continue;
            }

            double weight = e2.second;
            Eigen::Vector2d neighbor_position = landmark_vec_[lm_id_2_idx_map.at(e2.first)].vec();

            // draw a line from the root node to the neighbor
            glLineWidth(weight*2.0 + 1);
            //std::cerr << "drawing line between nodes.." << std::endl;
            pangolin::glDrawLine(root_position, neighbor_position);
          }
          lm_idx ++;
        }
      }
    }

  }

  inline void glDraw2dCovariance(double x, double y, Eigen::Matrix2d C, double conf )
  {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(0.5 * (C + C.transpose()));
    Eigen::Matrix2d eigenvalues = es.eigenvalues().asDiagonal();
    Eigen::Matrix2d eigenvectors = es.eigenvectors();
    Eigen::Matrix2d A = eigenvectors * (eigenvalues * conf).sqrt();

    const int N = 100;  // number of vertices
    Eigen::MatrixXd pts(N, 2);
    double inc = 2 * M_PI / N;
    for(size_t i = 0; i < N; ++i) {
      pts.row(i) = Eigen::Vector2d(std::cos(i * inc), std::sin(i * inc));
    }
    pts = pts * A;

    GLfloat verts[N*2];
    for(size_t i = 0; i < N; ++i ){
      verts[i*2] = x + (float)pts.row(i)[0];
      verts[i*2+1] = y + (float)pts.row(i)[1];
    }

    glVertexPointer(2, GL_FLOAT, 0, verts);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_LINE_LOOP, 0, N);
    glDisableClientState(GL_VERTEX_ARRAY);
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

  void SetColorBasedOnPersistence(bool set) {
    color_based_on_persistence_  = set;
  }

  void SetShowPersistenceLabels(bool show) {
    draw_persistence_labels_ = show;
  }

  void SetShowPersistenceWeights(bool show) {
    draw_persistence_weights_ = show;
  }

  void SetShowLandmarkId(bool show) {
    draw_landmark_id_ = show;
  }

  void SetSelectedLm(uint64_t lm_id){
    selected_lm_id_ = lm_id;
  }

  void ClearSelectedLms(){
    selected_lm_id_ = 1e10;
  }

  void SetPersistenceWeights(chameleon::FeaturePersistenceWeightsMapPtr weights) {
    persistence_weights_ = weights;
  }

private:
  std::vector<chameleon::Landmark> landmark_vec_;
  std::vector<SceneGraph::GLText> landmark_labels_vec_;
  chameleon::FeaturePersistenceWeightsMapPtr persistence_weights_;
  Eigen::Vector4f map_color_;
  bool color_based_on_persistence_;
  bool draw_persistence_labels_;
  bool draw_persistence_weights_;
  bool draw_landmark_id_;
  bool draw_variance_;
  uint64_t selected_lm_id_;
};
