#pragma once

#include "Eigen/Eigen"
#include "alined/state.hpp"


class SensorModel{

public:

  State updateState(const State &state);

  Eigen::MatrixXd updateCovariance();

  void setCovariance(const Eigen::MatrixXd &cov);


private:


protected:

  Eigen::MatrixXd covariance_;

  Eigen::MatrixXd jacobian_;




};
