#include "alined/sensor_model.hpp"

void SensorModel::setCovariance(const Eigen::MatrixXd &cov){

  covariance_ = cov;

}

State SensorModel::updateState(const State &state){

}

Eigen::MatrixXd SensorModel::updateCovariance(){

}
