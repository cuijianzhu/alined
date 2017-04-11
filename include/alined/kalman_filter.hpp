#pragma once

#include <functional>
#include "Eigen/Core"
#include "alined/motion_model.hpp"
#include "alined/sensor_model.hpp"
#include <memory>
#include <assert.h>

#define KF_SENSOR_1 0
#define KF_SENSOR_2 1
#define KF_SENSOR_3 2
#define KF_SENSOR_4 3
#define KF_SENSOR_5 4

#define KF_USE_QUATERNIONS    1
#define KF_USE_ANGLE_AXIS     2
#define KF_CONSTANT_VELOCITY  4


class KalmanFilter{

public:

  // Default constructor not allowed
  //KalmanFilter() = delete;

  // Copy constructor not allowed
  //KalmanFilter(const KalmanFilter&) = delete;


  KalmanFilter(int64_t config);
  ~KalmanFilter();

  void exit();

  void setInitialState(const ekf::State& state);
  void setCovarianceMM(const Eigen::MatrixXd& cov_mm);
  void setCovarianceMMByVector(const Eigen::VectorXd& cov_mm);
  void setNoiseVariances(const Eigen::VectorXd& noise);
  void setNoiseVarianceSM(const Eigen::MatrixXd& cov_sm, const u_char &sensor_id);
  void setMotionModel(MotionModel* mm);
  void pushSetSensorModel(SensorModel* sm);
  void predict(int64_t dt);
  void update(const u_char &sensor_id, const ekf::State &state);
  void printCovariance();
  void printState();
  ekf::State getState();
  int64_t getDT(int64_t time_now);

protected:

  bool model_set_ = false;
  bool sensor_set_ = false;

  bool inline isModelSet(){return model_set_;}
  bool inline isSensorSet(){return sensor_set_;}

  int64_t old_timestamp_;

  MotionModel* mm_;
  std::vector<SensorModel*> sm_;



};
