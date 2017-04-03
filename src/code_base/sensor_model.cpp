#include "alined/sensor_model.hpp"

void SensorModel::setCovariance(const Eigen::MatrixXd &cov){

  covariance_ = cov;

}

void SensorModel::updateState(){

}

void SensorModel::updateCovariance(){

}
