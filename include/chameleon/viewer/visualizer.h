// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <thread>
#include <mutex>
#include "chameleon/types.h"
#include "chameleon/util.h"
#include "chameleon/viewer/gl_landmark.h"
#include "chameleon/viewer/gl_robot.h"
#include "chameleon/viewer/gl_path_abs.h"
#include "chameleon/viewer/gl_map.h"
#include "chameleon/viewer/gl_data_associations.h"
#include "chameleon/viewer/gl_observations.h"
#include "chameleon/data_reader.h"

#include "fmt/format.h"

/*-----GUI Includes-----------*/
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/GLDynamicGrid.h>
/*----------------------------*/

namespace chameleon
{

class Visualizer {
public:
  struct ViewerOptions {
  public:
    std::string window_name = "Viewer";
    int window_height = 768;
    int window_width = 1024;
    unsigned int panel_size = 180;
    bool start_running = true;
  };

  // Struct for passing data from the application to the viewer
  struct ViewerData {
    ViewerData() {
      ground_truth_robot_poses = std::make_shared<RobotPoseVector>();
      noisy_robot_poses = std::make_shared<RobotPoseVector>();
      ground_truth_map = std::make_shared<LandmarkVector>();
      data_associations = std::make_shared<DataAssociationResults>();
      feature_persistence_weights_map = std::make_shared<FeaturePersistenceWeightsMap>();
    }

    ///
    /// \brief AddData > Adds simulated data to the visualizer
    /// \param data -> data to display
    ///
    void AddData(const RobotData& data) {
      ground_truth_robot_poses->push_back(data.debug.ground_truth_pose);
      noisy_robot_poses->push_back(data.debug.noisy_pose);
      if (data.debug.ground_truth_map != nullptr) {
        ground_truth_map = data.debug.ground_truth_map;
      }
      if(data.debug.feature_persistence_weights_map != nullptr) {
        feature_persistence_weights_map = data.debug.feature_persistence_weights_map;
      }
      ground_truth_observation_map.insert({data.index, data.debug.noise_free_observations});
      noisy_observation_map.insert({data.index, data.debug.noisy_observations});
    }

    void AddData(const DataReader::G2oData& data) {

      for (const auto& e : data.landmarks) {
        ground_truth_map->push_back(*(e.second));
      }
    }

    ///
    /// \brief AddData > Adds outputs from the estimator
    /// \param data > inferred data
    ///
    void AddData(const EstimatedData& data) {
      estimated_landmarks = data.landmarks;
      estimated_poses = data.states;
      *data_associations = data.data_association;
    }

    typedef std::shared_ptr<ViewerData> Ptr;
    RobotPoseVectorPtr ground_truth_robot_poses;
    RobotPoseVectorPtr noisy_robot_poses;
    LandmarkVectorPtr ground_truth_map;
    RangeFinderObservationVectorMap ground_truth_observation_map;
    RangeFinderObservationVectorMap noisy_observation_map;
    FeaturePersistenceWeightsMapPtr feature_persistence_weights_map;

    // estimated quantities
    LandmarkPtrMap estimated_landmarks;
    StatePtrMap estimated_poses;
    DataAssociationResults::Ptr data_associations;
  };

  // variables that will be displayed in the GUI
  struct DebugGUIVariables {
    std::unique_ptr<pangolin::Var<bool>> show_gt;
    std::unique_ptr<pangolin::Var<bool>> show_data_assoc;
    std::unique_ptr<pangolin::Var<bool>> show_observations;
    std::unique_ptr<pangolin::Var<bool>> show_gt_observations;
    std::unique_ptr<pangolin::Var<bool>> show_landmarks;
    std::unique_ptr<pangolin::Var<bool>> show_odometry;
    std::unique_ptr<pangolin::Var<bool>> show_estimated;
    std::unique_ptr<pangolin::Var<bool>> show_variance;
    std::unique_ptr<pangolin::Var<bool>> do_SLAM;
    std::unique_ptr<pangolin::Var<bool>> do_Localization;
    std::unique_ptr<pangolin::Var<bool>> reset;
    std::unique_ptr<pangolin::Var<bool>> show_prob_labels;
    std::unique_ptr<pangolin::Var<bool>> show_lm_ids;
    std::unique_ptr<pangolin::Var<bool>> color_lms;
    std::unique_ptr<pangolin::Var<bool>> draw_persistence_weights;
    std::unique_ptr<pangolin::Var<int>> plot_idx;
    std::unique_ptr<pangolin::Var<double>> prob_missed_detect;
    std::unique_ptr<pangolin::Var<double>> prob_false_detect;
  };

  struct GuiVars {
    pangolin::OpenGlRenderState camera;
    pangolin::OpenGlRenderState camera3d;
    SceneGraph::GLSceneGraph scene_graph;  // Scene Graph to hold GLObjects and realtive transforms
    std::unique_ptr<SceneGraph::HandlerSceneGraph> handler;
    SceneGraph::AxisAlignedBoundingBox aa_bounding_box;
    std::unique_ptr<pangolin::View> world_view_ptr;
    std::unique_ptr<pangolin::View> panel_view_ptr;
    std::unique_ptr<pangolin::View> multi_view_ptr;

    // plotting views
    std::unique_ptr<pangolin::DataLog> log_ptr;
    std::unique_ptr<pangolin::Plotter> plotter_ptr;

    // scene graph objects
    std::unique_ptr<SceneGraph::GLLight> light;
    std::unique_ptr<SceneGraph::GLDynamicGrid> dynamic_grid;  // Grid object to be the world plane
    std::unique_ptr<GLPathAbs> gt_robot_path;
    std::unique_ptr<GLPathAbs> noisy_robot_path;
    std::unique_ptr<GLMap> ground_truth_map;
    std::unique_ptr<GLObservations> ground_truth_observations;
    std::unique_ptr<GLObservations> noisy_observations;
    std::unique_ptr<GLDataAssociations> data_associations;
    // estimated quantities
    std::unique_ptr<GLMap> estimated_map;
    std::unique_ptr<GLPathAbs> estimated_robot_path;

    DebugGUIVariables ui;  // user interface
  };

  Visualizer(const ViewerOptions& options);

  void SetData(ViewerData::Ptr data);
  // which indeces from the data should we add to the display
  bool AddIndices(std::vector<size_t> indices);

  void RequestFinish();
  bool IsFinished();
  bool IsStepping();
  bool SetStepping(bool stepping);
  void SetReset();
  bool IsRunning();
  bool IsResetRequested();
  const DebugGUIVariables& GetDebugVariables();
  std::vector<uint64_t> GetLandmarksToBeRemoved();
  std::vector<uint64_t> ChangeLandmarks();


private:

  enum class ProjectionMatrixTypes {
    Perspective,
    Orthographic
  };

  void InitGui();
  void SwitchProjection(ProjectionMatrixTypes type);
  void AddLandmarks();
  void Run();
  void SetFinish();
  bool CheckFinish();
  void RequestReset();
  void AddObjectsToSceneGraph();
  void ResetSceneGraph();
  void UpdatePlotters();

  const ViewerOptions& options_;
  bool single_step_ = false;
  bool running_ = false;
  bool reset_requested_ = false;
  std::vector<uint64_t> landmarks_to_be_removed_;
  std::vector<uint64_t> landmarks_to_be_changed_;
  bool change_landmarks_ = false;


  GuiVars gui_vars_;
  ViewerData::Ptr data_;
  std::unique_ptr<std::thread> viewer_thread_;
  std::mutex status_mutex_;
  std::mutex data_mutex_;
  bool finished_ = false;
  bool finish_requested_ = false;
  size_t remove_lm_idx_ = 18;
};

}  // namespace chameleon
