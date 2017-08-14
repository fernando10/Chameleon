// Copyright 2017 Toyota Research Institute.  All rights reserved.
//

#include <memory>
#include <iostream>
#include <thread>
#include <glog/logging.h>
#include "chameleon/data_generator.h"
#include "chameleon/util.h"
#include "chameleon/math_utils.h"
#include "chameleon/viewer/gl_landmark2.h"

/*-----GUI Includes-----------*/
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/GLDynamicGrid.h>
/*----------------------------*/

using chameleon::Distribution;
using chameleon::MultivariateNormalVariable;


typedef Eigen::Vector2d RobotPose;
typedef Eigen::Vector2d Landmark;
typedef Eigen::Vector2d Measurement;
typedef Eigen::Matrix2d Covariance;

struct GuiVars {
  pangolin::OpenGlRenderState camera;
  pangolin::OpenGlRenderState camera3d;
  SceneGraph::GLSceneGraph scene_graph;  // Scene Graph to hold GLObjects and realtive transforms
  std::unique_ptr<SceneGraph::HandlerSceneGraph> handler;
  SceneGraph::AxisAlignedBoundingBox aa_bounding_box;
  std::unique_ptr<pangolin::View> world_view_ptr;
  std::unique_ptr<pangolin::View> multi_view_ptr;

  // scene graph objects
  std::unique_ptr<SceneGraph::GLLight> light;
  std::unique_ptr<SceneGraph::GLDynamicGrid> dynamic_grid;  // Grid object to be the world plane

};

GuiVars gui_vars_;

static constexpr double D_max = 0.0506;  // 0.975 confidence level for a 2-DoF Chi^2 distribution

// tracks our full sate (robot position, landmark 1 and landmark 2)
struct State {
  static constexpr size_t kIndexRobot = 0;
  static constexpr size_t kIndexLM1 = 2;
  static constexpr size_t kIndexLM2 = 4;

  State(Distribution d) : dist(d) {}

   Distribution Marginal(size_t index) {
     return Distribution(dist.mean.segment<2>(index), dist.cov.block<2, 2>(index, index));
   }

  Distribution dist;
};


double mahalanobis_distance(Eigen::VectorXd val, Distribution dist) {
  Eigen::VectorXd innovation = val - dist.mean;
  return innovation.transpose() * dist.cov.inverse() * innovation;
}


double ComputeAssociationCost(size_t& lm_index, Measurement& meas, Covariance& meas_cov, State& state) {
  Eigen::Matrix<double, 2, 6> H;
  H.setZero();
  H.block<2, 2>(0, State::kIndexRobot) = -Covariance::Identity();

  if (lm_index == 1) {
    Measurement predict = state.Marginal(State::kIndexLM1).mean - state.Marginal(State::kIndexRobot).mean;
    H.block<2, 2>(0, State::kIndexLM1) = Covariance::Identity();
    Covariance C = H * state.dist.cov * H.transpose() + meas_cov;
    return mahalanobis_distance(meas, Distribution(predict, C));
  }
  else if (lm_index == 2) {
    Measurement predict = state.Marginal(State::kIndexLM2).mean - state.Marginal(State::kIndexRobot).mean;
    H.block<2, 2>(0, State::kIndexLM2) = Covariance::Identity();
    Covariance C = H * state.dist.cov * H.transpose() + meas_cov;
    return mahalanobis_distance(meas, Distribution(predict, C));
  }
  return D_max;
}

void AddObjectsToSceneGraph() {
  // create scene graph objects
  gui_vars_.light = chameleon::util::make_unique<SceneGraph::GLLight>(10, 10, -1000);
  gui_vars_.scene_graph.AddChild(gui_vars_.light.get());
  gui_vars_.dynamic_grid = chameleon::util::make_unique<SceneGraph::GLDynamicGrid>();
  gui_vars_.scene_graph.AddChild(gui_vars_.dynamic_grid.get());
}

void InitGui() {
  int width = 1024;
  int height = 768;
  // Create a window.
  pangolin::CreateWindowAndBind("Demo", width, height);

  // setup camera
  gui_vars_.camera.SetModelViewMatrix(pangolin::ModelViewLookAt(10, 3, -15, 10, 0, 0, pangolin::AxisNegZ));
  gui_vars_.camera.SetProjectionMatrix(pangolin::ProjectionMatrix(width, height,
                                                                  420, 420, width/2.,height/2. , 0.01, 5000));

  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  // Reset background color to black.
  glClearColor(0, 0, 0, 1);

  AddObjectsToSceneGraph();

  // create view that will contain the robot/landmarks
  gui_vars_.world_view_ptr.reset(&pangolin::CreateDisplay()
                                 .SetAspect(-(float)width/(float)height)
                                 .SetBounds(0, 1.0, 0.0, 1.0));

  // create handler to allow user input to update modelview matrix and flow through to the scene graph
  gui_vars_.handler.reset(new SceneGraph::HandlerSceneGraph(gui_vars_.scene_graph, gui_vars_.camera,
                                                            pangolin::AxisNegZ));

  gui_vars_.world_view_ptr->SetHandler(gui_vars_.handler.get());
  gui_vars_.world_view_ptr->SetDrawFunction(SceneGraph::ActivateDrawFunctor(gui_vars_.scene_graph,
                                                                            gui_vars_.camera));

  // create a container view in case we want to have multiple views on the rigth side (maybe add some plots, etc)
  gui_vars_.multi_view_ptr.reset(&pangolin::Display("multi")
                                 .SetBounds(0.0, 1.0, 0.0, 1.0)
                                 .SetLayout(pangolin::LayoutVertical));

  // add the world view
  gui_vars_.multi_view_ptr->AddDisplay(*(gui_vars_.world_view_ptr));

}


int main(int argc, char **argv) {
  srand(0);  // make things deterministic

  // initialize logging/flag parsing
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  FLAGS_stderrthreshold = 0;

  // create our current estimate on the state
  Eigen::Matrix<double, 6, 1> mean;
  mean.segment<2>(State::kIndexRobot) = RobotPose(1., 1.);
  mean.segment<2>(State::kIndexLM1) = Landmark(1., 5.);
  mean.segment<2>(State::kIndexLM2) = Landmark(2.5, 5.);

  // crate state covariance
  Eigen::Matrix<double, 6, 6> S;
  S.block<2, 2>(State::kIndexRobot, State::kIndexRobot) = chameleon::Covariance2d(0.1, 0.05, 0);  // pose variance
  S.block<2, 2>(State::kIndexLM1, State::kIndexLM1) = chameleon::Covariance2d(0.2, 0.2, -0.4);  // landmark covariance
  S.block<2, 2>(State::kIndexLM2, State::kIndexLM2) = chameleon::Covariance2d(0.1, 0.3, -0.4);  // landmark covariance

  // create the distribution we will draw our true state from
  State state(Distribution(mean, S));
  MultivariateNormalVariable state_var(state.dist);

  // sample the state distribution to get our true state
  Eigen::VectorXd sample_state = state_var();
  RobotPose robot_true = sample_state.segment<2>(State::kIndexRobot);
  Landmark lm1_true = sample_state.segment<2>(State::kIndexLM1);
  Landmark lm2_true = sample_state.segment<2>(State::kIndexLM2);

  // and generate noisy observations from our ground truth state (enforcing independent measurements here)
  Eigen::Matrix2d R = chameleon::Covariance2d(0.1, 0.1, 0);
  MultivariateNormalVariable z1(R, lm1_true - robot_true );
  MultivariateNormalVariable z2(R, lm2_true  - robot_true);

  // sample from the measurement distributions
  Measurement meas_1 = z1();
  Measurement meas_2 = z2();

//  std::vector<size_t> persistence = {0, 1};
//  std::vector<std::tuple<size_t, size_t>> persistence_cross;
//  chameleon::cartesian_product(persistence, std::back_inserter(persistence_cross));

  // compute the cost of all the leaves of the interpretation tree
  std::vector<size_t> association = {0, 1, 2};
  std::vector<std::tuple<size_t, size_t>> association_cross;
  chameleon::cartesian_product(association, association, std::back_inserter(association_cross));

  for (auto&& e : association_cross) {
    if (std::get<0>(e) != 0 && std::get<0>(e) == std::get<1>(e)) {
      // repeated assignments to a landmark are not allowed
      continue;
    }

    double log_likelihood_total_cost = ComputeAssociationCost(std::get<0>(e), meas_1, R, state) +
                                       ComputeAssociationCost(std::get<1>(e), meas_2, R, state);

    LOG(INFO) << "Association: (" <<std::get<0>(e) <<", " << std::get<1>(e)<< ") : "  << log_likelihood_total_cost;
  }

  // plot stuff
  InitGui();

  // add robot pose
  RobotPose current_pose_estimate = state.Marginal(State::kIndexRobot).mean;
  SceneGraph::GLAxis robot;
  robot.SetPosition(current_pose_estimate[0], current_pose_estimate[1], 0);
  gui_vars_.scene_graph.AddChild(&robot);

  // add landmarks
  double lm_scale = 1;
  GLLandmark2 landmark1;
  landmark1.SetPosition(state.Marginal(State::kIndexLM1).mean[0], state.Marginal(State::kIndexLM1).mean[1], 0);
  landmark1.SetScale(lm_scale);
  landmark1.SetCovariance(state.Marginal(State::kIndexLM1).cov);
  landmark1.DrawCovariance(true);
  GLLandmark2 landmark2;
  landmark2.SetPosition(state.Marginal(State::kIndexLM2).mean[0], state.Marginal(State::kIndexLM2).mean[1], 0);
  landmark2.SetScale(lm_scale);
  gui_vars_.scene_graph.AddChild(&landmark1);
  gui_vars_.scene_graph.AddChild(&landmark2);

  // add the robot to the scene graph
  while (!pangolin::ShouldQuit()) {

    // clear whole screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    pangolin::FinishFrame();


    // Pause for 1/60th of a second
    std::this_thread::sleep_for(std::chrono::microseconds(1000 / 60));
  }



  return 0;
}

