#include "alined/kalman_filter.hpp"

KalmanFilter::KalmanFilter(int64_t config)
  :old_timestamp_(0){

  if((config & KF_USE_QUATERNIONS) == KF_USE_QUATERNIONS){

  }
  else if((config & KF_USE_ANGLE_AXIS) == KF_USE_ANGLE_AXIS){

  }

}


KalmanFilter::~KalmanFilter(){

}

void KalmanFilter::exit(){

}

void KalmanFilter::setMotionModel(MotionModel *mm){

  mm_ = mm;

  model_set_ = true;
}

void KalmanFilter::setInitialState(const ekf::State& state){

  if(!model_set_){
    std::cout<< "Model was not set. Use SetMotionModel(MotionModel *mm) first!\n";
    throw;
  }
  mm_->setInitialState(state);

}

void KalmanFilter::setCovarianceMM(const Eigen::MatrixXd& cov_mm){

  if(!model_set_){
    std::cout<< "Model was not set. Use SetMotionModel(MotionModel *mm) first!\n";
    throw;
  }

  mm_->setCovariance(cov_mm);

}

void KalmanFilter::setCovarianceMMByVector(const Eigen::VectorXd& cov_mm){

  if(!model_set_){
    std::cout<< "Model was not set. Use SetMotionModel(MotionModel *mm) first!\n";
    throw;
  }

  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(cov_mm.rows(),cov_mm.rows());
  cov.diagonal() = cov_mm;

  mm_->setCovariance(cov);

}

void KalmanFilter::setNoiseVariances(const Eigen::VectorXd &noise){

  mm_->setNoiseVariances(noise);

}

void KalmanFilter::setNoiseVarianceSM(const Eigen::MatrixXd& cov_sm, const u_char &sensor_id){

  if(!sensor_set_){
    std::cout<< "Sensor was not set. Use pushSetSensorModel(SensorModel *sm) first!\n";
    throw;
  }

  sm_[sensor_id]->setNoiseVariance(cov_sm);

}

void KalmanFilter::pushSetSensorModel(SensorModel *sm){

  sm_.push_back(sm);

  sensor_set_ = true;
}

void KalmanFilter::predict(int64_t dt){

  assert(model_set_);

  mm_->propagateState(dt);
  mm_->propagateCovariance(dt);
  old_timestamp_ = old_timestamp_+dt;

}

void KalmanFilter::update(const u_char &sensor_id, const ekf::State &state){

  assert(sensor_set_);

  mm_->setState(sm_[sensor_id]->updateState(mm_->getState(),mm_->getCovariance(), state));
  mm_->setCovariance(sm_[sensor_id]->updateCovariance());

}

void KalmanFilter::printCovariance(){
  std::cout << "Covariance Matrix:\n\n" << mm_->getCovariance() << "\n\n";
}

void KalmanFilter::printState(){
  ekf::State state = mm_->getState();
  std::cout << "State:\nPosition:\n" << state.position() << "\n\nVelocity:\n" << state.velocity()*1e+9 << "\n\nAngular Velocity:\n" << state.ang_vel()*1e+9 <<"\n\n";
}

int64_t KalmanFilter::getDT(int64_t time_now){

  return (time_now - old_timestamp_==0)?1:(time_now - old_timestamp_);
}

ekf::State KalmanFilter::getState(){
  return mm_->getState();
}
