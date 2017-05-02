#pragma once

#include "Eigen/Eigen"
#include "alined/state.hpp"


class SensorModel{

public:

  /*!
   * \brief update state based on measurement
   * \param state
   * \param covariance
   * \param update_state
   * \return
   */
  ekf::State updateState(ekf::State state,const Eigen::MatrixXd &covariance, ekf::State update_state);

  /*!
   * \brief update covariance
   * \return
   */
  Eigen::MatrixXd updateCovariance();

  /*!
   * \brief set noise covariance by matrix
   * \param cov
   */
  void setNoiseVariance(const Eigen::MatrixXd &cov);


private:


protected:

  Eigen::MatrixXd kalman_gain_;
  Eigen::MatrixXd covariance_;
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd noiseVar_;


};
