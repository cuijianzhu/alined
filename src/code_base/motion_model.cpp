#include "alined/motion_model.hpp"

MotionModel::MotionModel(){

  // Set motion model(TODO: later exchange with a customizeable motion model)
  f = std::bind(&MotionModel::constant_velocity_model,this,std::placeholders::_1);

}

MotionModel::State& getState(){
  return state_;
}

void MotionModel::setState(){

}


void MotionModel::setInitialState(State init_state){

  init_state_ = init_state;

}

void MotionModel::setCovariance(const Eigen::MatrixXd &cov){

  covariance_ = cov;

}


void MotionModel::propagateState(int64_t dt){

  // Propagate through nonlinear/linear function f
  f(dt);

}

void MotionModel::propagateCovariance(int64_t dt){

  // Change only entries of jacobian that vary with dt



}

void MotionModel::constant_velocity_model(int64_t dt){

  // T_new = T_old + dt*V_old
  state_.position() = state_.position().eval() + state_.velocity()*dt;

  // Q_new = Q_old * Q{Omega_old*dt}
  double norm_rot_vec = state_.ang_vel().norm();
  Eigen::Quaterniond innovation(Eigen::AngleAxisd(norm_rot_vec*dt, state_.ang_vel()/norm_rot_vec));
  state_.rotation() = state_.rotation().eval()*innovation;

  // V_new = V_old         no change necessary

  // Omega_new = Omega_old no change necessary

  //Create jacobian


}
