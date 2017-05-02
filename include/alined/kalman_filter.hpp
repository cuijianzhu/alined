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

/*!
 * \brief The KalmanFilter class implements a kalman filter based on certain motion model and sensor model.
 * At the moment measurement updates are done one sensor at a time, but this will be changed.
 */
class KalmanFilter{

public:


  KalmanFilter(int64_t config);
  ~KalmanFilter();

  void exit();

  /*!
   * \brief Set initial State of the Kalman Filter. The State is defined in the motion model class
   * \param state
   */
  void setInitialState(const ekf::State& state);

  /*!
   * \brief Set covariance matrix by elements
   * \param cov_mm
   */
  void setCovarianceMM(const Eigen::MatrixXd& cov_mm);

  /*!
   * \brief Set covariance matrix by vector holding diagonal elements
   * \param cov_mm
   */
  void setCovarianceMMByVector(const Eigen::VectorXd& cov_mm);

  /*!
   * \brief Set motion model noise by vector
   * \param noise
   */
  void setNoiseVariances(const Eigen::VectorXd& noise);

  /*!
   * \brief set sensor model noise of sensor with ID sensor_ID
   * \param cov_sm
   * \param sensor_id
   */
  void setNoiseVarianceSM(const Eigen::MatrixXd& cov_sm, const u_char &sensor_id);

  /*!
   * \brief Set the motion model
   * \param mm
   */
  void setMotionModel(MotionModel* mm);

  /*!
   * \brief Set one of several sensor models
   * \param sm
   */
  void pushSetSensorModel(SensorModel* sm);

  /*!
   * \brief Run a prediction step
   * \param dt
   */
  void predict(int64_t dt);

  /*!
   * \brief Run an update step using measurement from sensor with ID sensor_id
   * \param sensor_id
   * \param state
   */
  void update(const u_char &sensor_id, const ekf::State &state);

  /*!
   * \brief Print out covariance matrix
   */
  void printCovariance();

  /*!
   * \brief Print out state
   */
  void printState();

  /*!
   * \brief get state returned
   * \return
   */
  ekf::State getState();

  /*!
   * \brief get time difference to last timestamp
   * \param time_now
   * \return
   */
  int64_t getDT(int64_t time_now);

protected:


  /*!
   * \brief Check if model was set
   * \return
   */
  bool inline isModelSet(){return model_set_;}

  /*!
   * \brief Check if at least one sensor was set
   * \return
   */
  bool inline isSensorSet(){return sensor_set_;}

  bool model_set_ = false;                            // flag: 1 if model is set               0 else
  bool sensor_set_ = false;                           // flag: 1 if at least one sensor is set 0 else

  int64_t old_timestamp_;                             // Timestamp of last update

  MotionModel* mm_;                                   // Pointer to motion model
  std::vector<SensorModel*> sm_;                      // Vector of pointers to sensor models



};
