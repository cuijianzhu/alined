#pragma once
#include "Eigen/Eigen"
#include <functional>
#include <iostream>
#include "alined/state.hpp"

class MotionModel{

public:

  MotionModel();
  //MotionModel(const MotionModel&) = delete;


  ekf::State& getState();
  void setState(ekf::State state);

  void setInitialState(ekf::State init_state);
  void setCovariance(const Eigen::MatrixXd &cov);
  Eigen::MatrixXd getCovariance();
  void setNoiseVariances(const Eigen::VectorXd& noise);

  void propagateState(int64_t dt);
  void propagateCovariance(int64_t dt);



private:

protected:

 Eigen::MatrixXd covariance_;
 Eigen::MatrixXd jacobian_;
 Eigen::VectorXd noise_;

 void constant_velocity_model_propagate_state(int64_t dt);
 void constant_velocity_model_propagate_covariance(int64_t dt);

 std::function<void(int64_t dt)> f;
 std::function<void(int64_t dt)> P;

 ekf::State state_, init_state_;


};
