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

class KalmanFilter{

public:

  // Default constructor not allowed
  KalmanFilter() = delete;

  // Copy constructor not allowed
  KalmanFilter(const KalmanFilter&) = delete;


  KalmanFilter(int64_t config);

  void setInitialState(const MotionModel::State& state);
  void setCovarianceMM(const Eigen::MatrixXd& cov_mm);
  void setCovarianceSM(const Eigen::MatrixXd& cov_sm, const u_char &sensor_id);
  void setMotionModel(MotionModel* mm);
  void pushSetSensorModel(SensorModel* sm);
  void predict(int64_t dt);
  void update(const u_char &sensor_id);

protected:

  bool model_set_ = false;
  bool sensor_set_ = false;

  bool inline isModelSet(){return model_set_;}
  bool inline isSensorSet(){return sensor_set_;}

  MotionModel* mm_;
  std::vector<SensorModel*> sm_;



};
