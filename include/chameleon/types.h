// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <deque>
#include <iostream>
#include <vector>
#include <array>
#include <map>
#include <memory>
#include <sophus/se2.hpp>
#include <cstdint>
#include <Eigen/Core>
#include "chameleon/util.h"
#include "chameleon/math_utils.h"
#include "chameleon/interpolation_buffer.h"

static Eigen::IOFormat kCleanFmt(4, 0, ", ", ";\n", "", "");
static Eigen::IOFormat kLongFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "");
static Eigen::IOFormat kLongCsvFmt(Eigen::FullPrecision, 0, ", ", "\n", "", "");

namespace chameleon
{

//------------------------OBSERVATION TYPES------------------------//

typedef Eigen::Matrix<double, 2, 2> RangeFinderCovariance;
typedef Eigen::Matrix<double, 2, 2> OdometryReadingCovariance;


struct OdometryReading {
  static constexpr size_t kMeasurementDim = 2;
  static constexpr size_t kIndexVelocity = 0;
  static constexpr size_t kIndexOmega = 1;
  OdometryReading(): velocity(0), omega(0){}
  OdometryReading(double velocity, double omega): velocity(velocity), omega(omega){
  }

  Eigen::Vector2d vec() const {
    Eigen::Vector2d vec;
    vec[kIndexVelocity] = velocity;
    vec[kIndexOmega] = omega;
    return vec;
  }

  // Empirically tuned
  static constexpr double kVelocityStdDev = 0.01;  // [m/s]
  static constexpr double kOmegaStdDev = 0.008;  // [rad]
  static constexpr double kDt = 0.02;

  static OdometryReadingCovariance GetMeasurementCovariance() {
    return Covariance2d(kVelocityStdDev, kOmegaStdDev, 0);
  }


  OdometryReading operator+(const OdometryReading& rhs) const {
    return OdometryReading(velocity + rhs.velocity, omega + rhs.omega);
  }

  OdometryReading operator*(const double& rhs) const {
    return OdometryReading(velocity * rhs, omega * rhs);
  }

  double velocity;
  double omega;
};

struct RangeFinderReading {
  static constexpr size_t kMeasurementDim = 2;
  static constexpr size_t kIndexRange = 0;
  static constexpr size_t kIndexBearing = 1;
  RangeFinderReading(double theta, double range): theta(theta), range(range) {
  }

  RangeFinderReading(uint64_t lm_id_, double theta, double range): RangeFinderReading(theta, range) {
    lm_id = lm_id_;
  }

  Eigen::Vector2d vec() const {
    Eigen::Vector2d vec;
    vec[kIndexRange] = range;
    vec[kIndexBearing] = theta;
    return vec;
  }

  RangeFinderReading operator+(const RangeFinderReading& rhs) const {
    return RangeFinderReading(lm_id, theta + rhs.theta, range + rhs.range);
  }

  RangeFinderReading operator*(const double& rhs) const {
    return RangeFinderReading(lm_id, theta * rhs, range * rhs);
  }

  static constexpr double kBearingStdDev = 0.08;  // [rad]
  static constexpr double kRangeStdDev = 0.05;  // [m]

  static RangeFinderCovariance GetMeasurementCovariance() {
    return Covariance2d(kRangeStdDev, kBearingStdDev, 0);
  }


  RangeFinderReading operator+(const Eigen::Vector2d& rhs) const {
    return RangeFinderReading(lm_id, theta + rhs[kIndexBearing], range + rhs[kIndexRange]);
  }
  RangeFinderCovariance information;
  double theta;  // [rad]
  double range;  // [m]
  uint64_t lm_id = 0; // landmark id (only populated when generating simulated data
};



template<class T>
struct Observation {
  double time;
  T observation;

  Observation() : time(0.0) {
  }

  Observation(const double& time, const T& obs) :
    time(time), observation(obs) {
  }

  // Addition with another observation
  Observation<T> operator+(const Observation<T>& rhs) const {
    return Observation<T>(time, observation + rhs.observation);
  }

  // Multiplication with a scalar
  Observation<T> operator*(const double& rhs) const {
    return Observation<T>(time, observation * rhs);
  }

};



struct SE2OdometryMeasurement {
  static constexpr size_t kMeasurementDim = 3;
  Sophus::SE2d T;
  Eigen::Matrix3d information;

  SE2OdometryMeasurement() {}

  SE2OdometryMeasurement(Sophus::SE2d meas):
    T(meas) {
  }
};

struct OdometryMeasurement {
  static constexpr size_t kMeasurementDim = 3;
  double theta_1;
  double translation;
  double theta_2;

  OdometryMeasurement(): theta_1(0.), translation(0), theta_2(0.) {}

  OdometryMeasurement(double theta_1, double trans, double theta2):
    theta_1(theta_1), translation(trans), theta_2(theta2) {
  }
};


typedef Eigen::Matrix<double, OdometryMeasurement::kMeasurementDim,
OdometryMeasurement::kMeasurementDim> OdometryCovariance;

typedef std::vector<OdometryMeasurement> OdometryMeasurementVector;
typedef std::shared_ptr<std::vector<OdometryMeasurement>> OdometryMeasurementVectorPtr;
typedef Observation<RangeFinderReading> RangeFinderObservation;
typedef std::vector<RangeFinderObservation> RangeFinderObservationVector;
typedef std::map<size_t, RangeFinderObservationVector> RangeFinderObservationVectorMap;
typedef std::shared_ptr<InterpolationBufferT<RangeFinderObservation, double>> RangeFinderObservationBufferPtr;

typedef Observation<OdometryReading> OdometryObservation;
typedef std::vector<OdometryObservation>OdometryObservationVector;
typedef std::map<size_t, OdometryObservationVector> OdometryObservationVectorMap;
typedef InterpolationBufferT<OdometryObservation, double> OdometryObservationBuffer;
typedef std::shared_ptr<OdometryObservationBuffer> OdometryObservationBufferPtr;

//------------------------POSES AND LANDMARKS------------------------//

typedef Eigen::Matrix2d LandmarkCovariance;

struct Landmark {
  static constexpr size_t kLandmarkDim = 2;
  LandmarkCovariance covariance = LandmarkCovariance::Identity();
  uint64_t id;
  bool active = true;
  double persistence_prob = 0;

  Landmark(): covariance(LandmarkCovariance::Identity()), id(0) {
  }
  Landmark(uint64_t id_, double x, double y): Landmark(x, y) {
    id = id_;
  }
  Landmark(double x, double y): Landmark() {
    data_[0] = x;
    data_[1] = y;
  }

  // destructor
  ~Landmark() noexcept {}

  Landmark(const Landmark& lm) {
    id = lm.id;
    data_[0] = lm.x();
    data_[1] = lm.y();
    num_observations_ = lm.GetNumObservations();
    covariance = lm.covariance;
    active = lm.active;
    persistence_prob = lm.persistence_prob;
  }

  // Assignment -- copy
  Landmark& operator=(const Landmark& rhs) {
    data_ = rhs.vec();
    id = rhs.id;
    num_observations_ = rhs.num_observations_;
    covariance = rhs.covariance;
    active = rhs.active;
    persistence_prob = rhs.persistence_prob;
    return *this;
  }

  Landmark(Eigen::Vector2d vec): Landmark() {
    data_[0] = vec[0];
    data_[1] = vec[1];
  }

  double x() const { return data_[0]; }
  double y() const { return data_[1]; }
  double& x() { return data_[0]; }
  double& y() { return data_[1]; }

  void SetPosition(const double x, const double y) {
    data_[0] = x;
    data_[1] = y;
  }

  void AddObservation() {
    num_observations_++;
  }

  uint64_t GetNumObservations() const {
    return num_observations_;
  }

  Eigen::Vector2d vec() const {
    return data_;
  }

  friend std::ostream& operator<<(std::ostream& os, const Landmark& lm) {
    os << "( " << lm.x() << ", " << lm.y() << " )";
    return os;
  }

  // Should only be used by ceres cost function
  double* data() { return data_.data(); }

private:
  Eigen::Vector2d data_ = Eigen::Vector2d::Zero();
  uint64_t num_observations_ = 0;
};


typedef Eigen::Matrix3d PoseCovariance;
struct RobotPose {
  Sophus::SE2d pose;
  PoseCovariance covariance;
  double field_of_view = Deg2Rad(90);  // [rad]
  double range = 10;  // [m]
  static constexpr size_t kIndexTheta = 0;
  static constexpr size_t kIndexTrans = 1;

  // the covariance matrix is first translation then rotation
  // this follows the order retured by Sophus::SE2::log (x, y, theta)
  static constexpr size_t kIndexThetaCov = 2;
  static constexpr size_t kIndexTransCov = 0;

  // Default constructor
  RobotPose(): covariance(PoseCovariance::Identity()) {
  }

  ~RobotPose() noexcept {}

  // Construct from angle and position
  RobotPose(const double theta, const Eigen::Vector2d position) : RobotPose() {
    pose = Sophus::SE2d(theta, position);
  }

  // Construct from an SE2d transform
  RobotPose(const Sophus::SE2d p): RobotPose() {
    pose = p;
  }

  // Construct from an SO2d and position
  RobotPose(const Sophus::SO2d rot, const Eigen::Vector2d pos): RobotPose() {
    pose = Sophus::SE2d(rot, pos);
  }

  // Construct from an angle and x, y
  RobotPose(const double theta, const double x, const double y): RobotPose() {
    pose = Sophus::SE2d(theta, Eigen::Vector2d(x, y));
  }

  // Construct from another RobotPose
  RobotPose(const RobotPose& other) {
    pose = other.pose;
    covariance = other.covariance;
  }

  // Multiplication with another robot pose
  RobotPose operator*(const RobotPose& rhs) const {
    return RobotPose(pose * rhs.pose);
    // TODO: propagate covariance
  }

  RangeFinderReading Predict(const Landmark& lm, Eigen::MatrixXd* jacobian = nullptr) const {
    return Predict(this->pose, lm, jacobian);
  }
  ///
  /// \brief Predict
  /// Predicts a measurement based on the landmark
  /// \param lm landmark in world coordinates
  /// \param jacobian -> optinally fill out the prediction jacobian w.r.t the current pose and the landmark
  /// \return
  ///
  RangeFinderReading Predict(const Sophus::SE2d& pose, const Landmark& lm, Eigen::MatrixXd* jacobian = nullptr) const {
    Eigen::Vector2d lm_r = util::DeHomogenizeLandmark<double>(pose.inverse().matrix() *
                                                              util::HomogenizeLandmark<double>(lm.vec()));

    double range_pred = lm_r.norm();
    double bearing_pred = AngleWraparound<double>(std::atan2(lm_r.y(), lm_r.x()));

    if (jacobian != nullptr) {
      // jacobian requested
      jacobian->resize(2, 5); // w.r.t current state (3) and landmark in world frame (2)

      // derivative w.r.t landmark
      (*jacobian)(0, 3) = lm_r.x() / lm_r.norm();
      (*jacobian)(0, 4) = lm_r.y() / lm_r.norm();
      (*jacobian)(1, 3) = -lm_r.y() / Square(lm_r.norm());
      (*jacobian)(1, 4) = lm_r.x() / Square(lm_r.norm());

      // derivative w.r.t state (x, y, theta)
      (*jacobian)(0, 0) = -lm_r.x() / lm_r.norm();
      (*jacobian)(0, 1) = -lm_r.y() / lm_r.norm();
      (*jacobian)(0, 2) = 0;
      (*jacobian)(1, 0) = lm_r.y() / Square(lm_r.norm());
      (*jacobian)(1, 1) = -lm_r.x() / Square(lm_r.norm());
      (*jacobian)(1, 2) = -1;
      //FiniteDifferencesPredictionJacobian(lm, jacobian);
    }
    return RangeFinderReading(bearing_pred, range_pred);
  }

  bool CheckPredictionJacobian(const Landmark& lm, double tol = 1e-8) const {
    Sophus::SE2d pose = this->pose;
    Landmark landmark = lm;

    Eigen::MatrixXd J;
    Predict(pose, landmark, &J);

    Eigen::MatrixXd J_num_diff;
    FiniteDifferencesPredictionJacobian(lm, &J_num_diff);

    //    std::cerr << " J: \n" << J << " \n J_num_diff:\n"  << J_num_diff << " \n diff: " << J - J_num_diff << std::endl;
    //    std::cerr << "norm: " << (J - J_num_diff).norm();
    return  (J - J_num_diff).norm() < tol;
  }

  void FiniteDifferencesPredictionJacobian(const Landmark& lm, Eigen::MatrixXd* J_out) const {
    Sophus::SE2d pose = this->pose;
    Landmark landmark = lm;

    Eigen::Matrix<double, 2, 5> J_num_diff;
    J_num_diff.setZero();
    double eps = 1e-6;
    for (size_t i = 0; i < 3; ++i) {
      // tweak pose
      Eigen::Matrix<double, 3, 1> delta;
      delta[i] = eps;
      Sophus::SE2d pose_plus = pose * Sophus::SE2d::exp(delta);
      RangeFinderReading res_p = Predict(pose_plus, landmark);

      delta[i] = -eps;
      Sophus::SE2d pose_minus = pose * Sophus::SE2d::exp(delta);
      RangeFinderReading res_m = Predict(pose_minus, landmark);

      J_num_diff.col(i) = (res_p.vec() - res_m.vec()) / (2 * eps);
    }

    for (size_t i = 0; i <2; ++i) {
      // tweak landmark
      Eigen::Matrix<double, 2, 1> delta;
      delta[i] = eps;
      Landmark lm_plus(landmark.vec() + delta);
      RangeFinderReading res_p = Predict(pose, lm_plus);

      delta[i] = -eps;
      Landmark lm_minus(landmark.vec() + delta);
      RangeFinderReading res_m = Predict(pose, lm_minus);

      J_num_diff.col(i+3) = (res_p.vec() - res_m.vec()) / (2 * eps);
    }

    J_out->resize(2, 5);
    J_out->setZero();
    *J_out = J_num_diff;
  }

  // Multiplication with an SE2d transform
  RobotPose operator*(const Sophus::SE2d& rhs) const {
    return RobotPose(pose * rhs);
  }

  // Multiplication with an landmark
  Landmark operator*(const Landmark& rhs) const {
    return Landmark(util::DeHomogenizeLandmark<double>(pose.matrix() * util::HomogenizeLandmark<double>(rhs.vec())));
  }

  // Assignment -- copy
  RobotPose& operator=(const RobotPose& rhs) {
    pose = rhs.pose;
    covariance = rhs.covariance;
    return *this;
  }

  const Eigen::Vector2d& translation() const {
    return pose.translation();
  }

  const Sophus::SO2d& SO2() const {
    return pose.so2();
  }

  Eigen::VectorXd vec() {
    Eigen::VectorXd ret(Sophus::SE2d::DoF);
    ret[kIndexTheta] = theta();
    ret[kIndexTrans] = x();
    ret[kIndexTrans+1] = y();
    return ret;
  }

  double theta() const { return pose.so2().log(); }

  // we don't store theta directly so can't easily return a reference like we do for x and y
  void SetTheta(const double theta_new) {
    pose.so2() = Sophus::SO2d(AngleWraparound<double>(theta_new));
  }

  double x() const { return pose.translation().x(); }
  double& x() { return pose.translation().x(); }

  double y() const { return pose.translation().y(); }
  double& y() { return pose.translation().y(); }

  // only to be used by ceres parameter block
  double* data() { return pose.data(); }

  friend std::ostream& operator<<(std::ostream& os, const RobotPose& robot) {
    os << "theta: " << Rad2Deg(robot.theta()) << ", position: " << robot.translation().transpose();
    return os;
  }

};

typedef std::shared_ptr<Landmark> LandmarkPtr;
typedef std::map<uint64_t, LandmarkPtr> LandmarkPtrMap;
typedef std::vector<Landmark> LandmarkVector;
typedef std::shared_ptr<LandmarkVector> LandmarkVectorPtr;
typedef std::vector<RobotPose> RobotPoseVector;
typedef std::shared_ptr<RobotPoseVector> RobotPoseVectorPtr;
typedef std::map<size_t, uint64_t> DataAssociationMap;


//------------------------DATA STRUCTURES------------------------//

//struct DebugData {
//  DebugData() {
//    ground_truth_map = std::make_shared<LandmarkVector>();
//    ground_truth_poses = std::make_shared<RobotPoseVector>();
//    noisy_poses = std::make_shared<RobotPoseVector>();
//    noise_free_odometry = std::make_shared<OdometryMeasurementVector>();
//    noisy_odometry = std::make_shared<OdometryMeasurementVector>();
//  }

//  RobotPoseVectorPtr ground_truth_poses;
//  RobotPoseVectorPtr noisy_poses;
//  OdometryMeasurementVectorPtr noise_free_odometry;
//  OdometryMeasurementVectorPtr noisy_odometry;
//  LandmarkVectorPtr ground_truth_map;
//  RangeFinderObservationVectorMap noise_free_observations;
//  RangeFinderObservationVectorMap noisy_observations;
//};

// debug data for a single timestep
struct DebugData {
  DebugData() {
    ground_truth_map = std::make_shared<LandmarkVector>();
    noisy_map = std::make_shared<LandmarkVector>();
  }

  RobotPose ground_truth_pose;
  RobotPose noisy_pose;
  OdometryMeasurement noise_free_odometry;
  OdometryMeasurement noisy_odometry;
  LandmarkVectorPtr ground_truth_map;
  LandmarkVectorPtr noisy_map;
  RangeFinderObservationVector noise_free_observations;
  RangeFinderObservationVector noisy_observations;
};

struct SimData {
  std::vector<size_t> times;
  DebugData debug;
};

struct VictoriaParkData {
  double timestamp;
  SE2OdometryMeasurement odometry;
  RangeFinderObservationVector observations;
};

struct RobotData {
  double timestamp;
  size_t index;
  DebugData debug;
  // this is all that is available to the estimation algorithm (below)
  OdometryMeasurement odometry;
  OdometryObservationVector odometry_readings;
  RangeFinderObservationVector observations;
};

//------------------------STATE ESTIMATION STUFF------------------------//
struct State {
  State():id(0), timestamp(0), fixed(false), active(true) {}
  static constexpr size_t kStateDim = Sophus::SE2d::DoF;
  uint64_t id;
  size_t robot_idx; // robot index in a multi-robot setting;
  RobotPose robot;
  double timestamp;
  bool fixed;
  bool active;
  double* data() { return robot.data(); }
  int DoF() { return robot.pose.DoF; }
};

struct Marginals {
  RobotPose robot;
  uint64_t pose_id;
  LandmarkPtrMap landmarks;
  std::map<std::pair<uint64_t, uint64_t>, Eigen::MatrixXd> covariances;
};

typedef std::shared_ptr<State> StatePtr;
typedef std::map<uint64_t, StatePtr> StatePtrMap;
typedef std::vector<StatePtr> StatePtrVector;
typedef std::multimap<uint64_t, uint64_t> State2Landmark_Multimap;
typedef std::multimap<uint64_t, uint64_t> Landmark2State_MultiMap;

struct DataAssociationResults {
  typedef std::shared_ptr<DataAssociationResults> Ptr;
  void Clear() {
    associations.clear();
    observations.clear();
  }

  DataAssociationMap associations;
  RangeFinderObservationVector observations;
};

struct EstimatedData {
  //const StatePtr state;
  StatePtrMap states;
  LandmarkPtrMap landmarks;
  DataAssociationResults data_association;
};

} // namespace chameleon
