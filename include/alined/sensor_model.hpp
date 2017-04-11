#pragma once

#include "Eigen/Eigen"
#include "alined/state.hpp"


class SensorModel{

public:

  ekf::State updateState(ekf::State state,const Eigen::MatrixXd &covariance, ekf::State update_state);

  Eigen::MatrixXd updateCovariance();

  void setNoiseVariance(const Eigen::MatrixXd &cov);


private:


protected:

  Eigen::MatrixXd kalman_gain_;
  Eigen::MatrixXd covariance_;
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd noiseVar_;


};
