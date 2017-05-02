#include "alined/sensor_model.hpp"
#include <iostream>

void SensorModel::setNoiseVariance(const Eigen::MatrixXd &cov){

  noiseVar_ = cov;

}

ekf::State SensorModel::updateState(ekf::State state,const Eigen::MatrixXd &covariance, ekf::State update_state){

  // Global Position and Orientation are independent of each other.
  // Therefore, we can update them separately

  ekf::State updated;

  covariance_ = covariance;

  // Position:

  // S = HPH' + R = P + R
  Eigen::Matrix3d inv_S_pos = (covariance_.block<3,3>(0,0) + noiseVar_.block<3,3>(0,0)).inverse();
  //std::cout << "inv_s_pos:\n\n"<< inv_S_pos <<"\n\n";

  // K = PH'S⁻¹
  kalman_gain_.resize(6,3);

  kalman_gain_.block<6,3>(0,0) = covariance_.block<6,3>(0,0)*inv_S_pos;

  updated.position() = state.position() + kalman_gain_.block<3,3>(0,0)*(update_state.position()-state.position());
  updated.velocity() = state.velocity() + kalman_gain_.block<3,3>(3,0)*(update_state.position()-state.position());

  //std::cout<<"R:\n\n" << noiseVar_ <<"\n\nP:\n\n"<<covariance_.block<3,3>(0,0)<<"\n\n";
  // Orientation:

  //TODO: orientation update...


  return updated;
}

Eigen::MatrixXd SensorModel::updateCovariance(){

  // Position update
  Eigen::MatrixXd cov_P = covariance_;

  Eigen::Matrix<double,6,3> i_minus_k = -kalman_gain_;
  i_minus_k.block<3,3>(0,0).diagonal() = i_minus_k.block<3,3>(0,0).eval().diagonal() + Eigen::Vector3d(1,1,1);

  cov_P.block<3,3>(0,0) = i_minus_k.block<3,3>(0,0)*covariance_.block<3,3>(0,0);
  cov_P.block<3,3>(0,3) = i_minus_k.block<3,3>(0,0)*covariance_.block<3,3>(0,3);
  cov_P.block<3,3>(3,0) = i_minus_k.block<3,3>(3,0)*covariance_.block<3,3>(0,0) + covariance_.block<3,3>(3,0);
  cov_P.block<3,3>(3,3) = i_minus_k.block<3,3>(3,0)*covariance_.block<3,3>(0,3) + covariance_.block<3,3>(3,3);

  // In case that due to numerical issues P is not symmetric, make it symmetric again
  cov_P = 0.5*(cov_P + cov_P.transpose()).eval();

  //std::cout << "Cov_P:\n\n" << cov_P<<"\n\n";
  return cov_P;
}
