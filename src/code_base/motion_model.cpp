#include "alined/motion_model.hpp"

#define KF_SPEED_NUMERICAL_LIMIT 0.0000000001

MotionModel::MotionModel(){

  // Set motion model(TODO: later exchange with a customizeable motion model)
  f = std::bind(&MotionModel::constant_velocity_model_propagate_state,this,std::placeholders::_1);
  P = std::bind(&MotionModel::constant_velocity_model_propagate_covariance,this,std::placeholders::_1);

}

State& MotionModel::getState(){
  return state_;
}

void MotionModel::setState(State state){

  state_ = state;

}


void MotionModel::setInitialState(State init_state){

  state_ = init_state;

}

void MotionModel::setCovariance(const Eigen::MatrixXd &cov){

  covariance_ = cov;
  std::cout << "Covariance_init:\n\n"<<covariance_<<"\n\n";

}

Eigen::MatrixXd MotionModel::getCovariance(){
  return covariance_;
}

void MotionModel::setNoiseVariances(const Eigen::VectorXd &noise){
  noise_ = noise;
}


void MotionModel::propagateState(int64_t dt){

  // Propagate through nonlinear/linear function f
  f(dt);

}

void MotionModel::propagateCovariance(int64_t dt){

  // Propagate through nonlinear function P
  P(dt);
}

void MotionModel::constant_velocity_model_propagate_covariance(int64_t dt){

  //Update Covariance

  /*
   * P_pos = [1 dt;0 1]*P_old*[1 0;dt 1];
   * P_noise = [dt 0;0 dt]N[dt 0;0 dt];
   *
   */

  //Position and Orientation covariances are updated separately as position jacobian doesn't
  //correlate with orientation jacobian

  int64_t dt_square = dt*dt;
  int64_t dt_2 = 2*dt;

  // IMPORTANT: The following order retains correct results without any unnecessary copies
  covariance_(3,0) = covariance_(3,0) + dt*covariance_(3,3);
  covariance_(4,1) = covariance_(4,1) + dt*covariance_(4,4);
  covariance_(5,2) = covariance_(5,2) + dt*covariance_(5,5);
  covariance_(0,0) = covariance_(0,0) + dt_2*covariance_(0,3) + dt_square*covariance_(3,3) + dt_square*noise_(0);
  covariance_(1,1) = covariance_(1,1) + dt_2*covariance_(1,4) + dt_square*covariance_(4,4) + dt_square*noise_(1);
  covariance_(2,2) = covariance_(2,2) + dt_2*covariance_(2,5)+ dt_square*covariance_(5,5) + dt_square*noise_(2);
  covariance_(3,3) = covariance_(3,3) + dt_square*noise_(3);
  covariance_(4,4) = covariance_(4,4) + dt_square*noise_(4);
  covariance_(5,5) = covariance_(5,5) + dt_square*noise_(5);
  covariance_(0,3) = covariance_(3,0);
  covariance_(1,4) = covariance_(4,1);
  covariance_(2,5) = covariance_(5,2);


  // Orientation Variance:

  std::cout << state_.innovation().w() <<" "<< state_.innovation().vec()<<"\n";
  // dQ/dq = [q{wdt}]_R
  Eigen::MatrixXd jac_ = Eigen::MatrixXd::Zero(4,7);
  jac_.block<4,4>(0,0).diagonal() = Eigen::Vector4d(state_.innovation().w(),state_.innovation().w(),state_.innovation().w(),state_.innovation().w());
  jac_.block<1,3>(0,1) = state_.innovation().vec().transpose();
  jac_.block<3,1>(1,0) = state_.innovation().vec();
  jac_(1,2) = state_.innovation().z();
  jac_(1,3) = -state_.innovation().y();
  jac_(2,1) = -state_.innovation().z();
  jac_(3,1) = state_.innovation().y();
  jac_(2,3) = state_.innovation().x();
  jac_(3,2) = -state_.innovation().x();

  // The jacobian w.r.t. the orientation is ugly in its derivation:
  // Taking the partial derivatives dq/dw = dq/dq_innov * dq_innov/dw
  // The first part is [q]_skew_Left the left skew matrix of the old quaternion
  // The second part are the partial derivatives of the innovation quaternion, which comes from
  // the famous Rodriguez formula (It's strongly recommended to use a symbolic solver for this task)
  // The interested reader (who might be spanish and go by the name of Ignacio) is referred to:
  // http://www.cs.cmu.edu/afs/cs.cmu.edu/user/spiff/www/moedit99/expmap.pdf

  double_t dt_half = 0.5*dt;

  double norm_rot_vec = state_.ang_vel().norm();
  double inv_norm_vec = 1.0/norm_rot_vec;
  double dt_half_normalized = dt_half * inv_norm_vec;

  double wx = state_.ang_vel()(0);
  double wy = state_.ang_vel()(1);
  double wz = state_.ang_vel()(2);

  Eigen::Vector3d dW_dXYZ;
  Eigen::Vector3d dX_dXYZ;
  Eigen::Vector3d dY_dXYZ;
  Eigen::Vector3d dZ_dXYZ;

  if(norm_rot_vec > KF_SPEED_NUMERICAL_LIMIT){

      //Case: omega bigger 0.0001

      double sine_term = inv_norm_vec*sin(norm_rot_vec*dt_half); // This has to be calculated manually, as the inverse of wx,wy,wz could cause numerical instability.

      Eigen::Matrix3d base = dt_half_normalized*state_.innovation().w()*state_.ang_vel()*state_.ang_vel().transpose();
      Eigen::Vector3d terms(wx*inv_norm_vec*state_.innovation().x(), wy*inv_norm_vec*state_.innovation().y(), wz*inv_norm_vec*state_.innovation().z());

      dW_dXYZ = -dt_half*state_.innovation().vec();
      dX_dXYZ = Eigen::Vector3d(base(0,0)-terms(0)+ sine_term, base(0,1)-terms(1), base(0,2)-terms(2));
      dY_dXYZ = Eigen::Vector3d(base(1,0)-terms(0), base(1,1)-terms(1)+ sine_term, base(1,2)-terms(2));
      dZ_dXYZ = Eigen::Vector3d(base(2,0)-terms(0), base(2,1)-terms(1), base(2,2)-terms(2)+ sine_term);
  }
  else{

    Eigen::Matrix3d base = ((dt_square*norm_rot_vec*norm_rot_vec/40)-1)*dt*dt_square/24*state_.ang_vel()*state_.ang_vel().transpose();
    double term = dt_half - dt*dt_square*norm_rot_vec*norm_rot_vec/48;

    dW_dXYZ = -dt_half*(0.5-dt_square*norm_rot_vec*norm_rot_vec/48.0)*state_.ang_vel();
    dX_dXYZ = Eigen::Vector3d(base(0,0) + term, base(0,1), base(0,2));
    dY_dXYZ = Eigen::Vector3d(base(1,0), base(1,1) + term, base(1,2));
    dZ_dXYZ = Eigen::Vector3d(base(2,0), base(2,1), base(2,2) + term);

  }

  jac_(0,4) = state_.rotation().w()*(dW_dXYZ(0)) - state_.rotation().x()*(dX_dXYZ(0)) - state_.rotation().y()*(dY_dXYZ(0)) - state_.rotation().z()*(dZ_dXYZ(0));
  jac_(1,4) = state_.rotation().w()*(dX_dXYZ(0)) + state_.rotation().x()*(dW_dXYZ(0)) + state_.rotation().y()*(dZ_dXYZ(0)) - state_.rotation().z()*(dY_dXYZ(0));
  jac_(2,4) = state_.rotation().w()*(dY_dXYZ(0)) - state_.rotation().x()*(dZ_dXYZ(0)) + state_.rotation().y()*(dW_dXYZ(0)) + state_.rotation().z()*(dX_dXYZ(0));
  jac_(3,4) = state_.rotation().w()*(dZ_dXYZ(0)) + state_.rotation().x()*(dY_dXYZ(0)) - state_.rotation().y()*(dX_dXYZ(0)) + state_.rotation().z()*(dW_dXYZ(0));

  jac_(0,5) = state_.rotation().w()*(dW_dXYZ(1)) - state_.rotation().x()*(dX_dXYZ(1)) - state_.rotation().y()*(dY_dXYZ(1)) - state_.rotation().z()*(dZ_dXYZ(1));
  jac_(1,5) = state_.rotation().w()*(dX_dXYZ(1)) + state_.rotation().x()*(dW_dXYZ(1)) + state_.rotation().y()*(dZ_dXYZ(1)) - state_.rotation().z()*(dY_dXYZ(1));
  jac_(2,5) = state_.rotation().w()*(dY_dXYZ(1)) - state_.rotation().x()*(dZ_dXYZ(1)) + state_.rotation().y()*(dW_dXYZ(1)) + state_.rotation().z()*(dX_dXYZ(1));
  jac_(3,5) = state_.rotation().w()*(dZ_dXYZ(1)) + state_.rotation().x()*(dY_dXYZ(1)) - state_.rotation().y()*(dX_dXYZ(1)) + state_.rotation().z()*(dW_dXYZ(1));

  jac_(0,6) = state_.rotation().w()*(dW_dXYZ(2)) - state_.rotation().x()*(dX_dXYZ(2)) - state_.rotation().y()*(dY_dXYZ(2)) - state_.rotation().z()*(dZ_dXYZ(2));
  jac_(1,6) = state_.rotation().w()*(dX_dXYZ(2)) + state_.rotation().x()*(dW_dXYZ(2)) + state_.rotation().y()*(dZ_dXYZ(2)) - state_.rotation().z()*(dY_dXYZ(2));
  jac_(2,6) = state_.rotation().w()*(dY_dXYZ(2)) - state_.rotation().x()*(dZ_dXYZ(2)) + state_.rotation().y()*(dW_dXYZ(2)) + state_.rotation().z()*(dX_dXYZ(2));
  jac_(3,6) = state_.rotation().w()*(dZ_dXYZ(2)) + state_.rotation().x()*(dY_dXYZ(2)) - state_.rotation().y()*(dX_dXYZ(2)) + state_.rotation().z()*(dW_dXYZ(2));


  covariance_.block<7,7>(6,6) = jac_*covariance_.block<7,7>(6,6).eval()*jac_.transpose();

  // Orientation Variance: Add noise variance
  // NOTE: It is clear that the noise applied on all 4 quaternion states doesn't represent the real system as it would have correlated noise
  //       between n1*||omega|| and n2*omega, which would transform into [cos(n1*||omega||), n2/n1*omega/||omega|| sin(n1*||omega||)]
  //       Nevertheless it can be seen as noise applied on perturbation impulse quaternions. Normalization will handle the problem, that the
  //       noise moves the quaternion out of the unit ball on S3x3
  covariance_.block<7,7>(6,6).diagonal() = covariance_.block<7,7>(6,6).diagonal().eval() + dt_square*noise_.block<7,1>(6,0);

}

void MotionModel::constant_velocity_model_propagate_state(int64_t dt){

  // T_new = T_old + dt*V_old
  state_.position() = state_.position().eval() + state_.velocity()*dt;

  // Q_new = Q_old * Q{Omega_old*dt}
  double norm_rot_vec = state_.ang_vel().norm();

  // For small angles of omega, the rotation is numerically unstable.
  // We therefore approximate the rotation using the Sinc function
  if(norm_rot_vec > KF_SPEED_NUMERICAL_LIMIT){
    state_.innovation() = Eigen::Quaterniond(Eigen::AngleAxisd(norm_rot_vec*dt, state_.ang_vel()/norm_rot_vec));
  }
  else{
    // We use the fact that a cos(theta-->0) is approximately 1. For the sine we use the 0th order taylor expansion
    state_.innovation() = Eigen::Quaterniond(1,dt*state_.ang_vel()(0)*0.5, dt*state_.ang_vel()(1)*0.5, dt*state_.ang_vel()(2)*0.5);
  }

  state_.rotation() = (state_.rotation()*state_.innovation());

  // V_new = V_old         no change necessary

  // Omega_new = Omega_old no change necessary



}
