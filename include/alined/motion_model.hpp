#pragma once
#include "Eigen/Eigen"
#include <functional>
#include <iostream>
#include "alined/state.hpp"

class MotionModel{

public:

  MotionModel();

  /*!
   * \brief get motion model state
   * \return
   */
  ekf::State& getState();

  /*!
   * \brief set motion model state
   * \param state
   */
  void setState(ekf::State state);

  /*!
   * \brief set initial state
   * \param init_state
   */
  void setInitialState(ekf::State init_state);

  /*!
   * \brief set covariance matrix
   * \param cov
   */
  void setCovariance(const Eigen::MatrixXd &cov);

  /*!
   * \brief get covariance matrix
   * \return
   */
  Eigen::MatrixXd getCovariance();

  /*!
   * \brief set noise covariance matrix by vector
   * \param noise
   */
  void setNoiseVariances(const Eigen::VectorXd& noise);

  /*!
   * \brief propagate state for dt nanoseconds
   * \param dt
   */
  void propagateState(int64_t dt);

  /*!
   * \brief propagate covariance for dt nanoseconds
   * \param dt
   */
  void propagateCovariance(int64_t dt);



private:

protected:

  /*!
  * \brief function for state propagation- hard coded a.t.m.
  * \param dt
  */
  void constant_velocity_model_propagate_state(int64_t dt);

  /*!
   * \brief function for covariance propagation- hard coded a.t.m.
   * \param dt
   */
  void constant_velocity_model_propagate_covariance(int64_t dt);

  // Functors
  std::function<void(int64_t dt)> f;
  std::function<void(int64_t dt)> P;

  Eigen::MatrixXd covariance_;
  Eigen::MatrixXd jacobian_;
  Eigen::VectorXd noise_;

  ekf::State state_, init_state_;


};
