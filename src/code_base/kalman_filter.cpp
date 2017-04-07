#include "alined/kalman_filter.hpp"

KalmanFilter::KalmanFilter(int64_t config){

}

KalmanFilter::~KalmanFilter(){

}

void KalmanFilter::exit(){

}

void KalmanFilter::setMotionModel(MotionModel *mm){

  mm_ = mm;

  model_set_ = true;
}

void KalmanFilter::setInitialState(const State& state){

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

void KalmanFilter::setCovarianceSM(const Eigen::MatrixXd& cov_sm, const u_char &sensor_id){

  if(!sensor_set_){
    std::cout<< "Sensor was not set. Use pushSetSensorModel(SensorModel *sm) first!\n";
    throw;
  }

  sm_[sensor_id]->setCovariance(cov_sm);

}

void KalmanFilter::pushSetSensorModel(SensorModel *sm){

  sm_.push_back(sm);

  sensor_set_ = true;
}

void KalmanFilter::predict(int64_t dt){

  assert(model_set_);

  mm_->propagateState(dt);
  mm_->propagateCovariance(dt);

}

void KalmanFilter::update(const u_char &sensor_id, const State &state){

  assert(sensor_set_);

  mm_->setState(sm_[sensor_id]->updateState(state));
  mm_->setCovariance(sm_[sensor_id]->updateCovariance());

}

void KalmanFilter::printCovariance(){
  std::cout << "Covariance Matrix:\n\n" << mm_->getCovariance() << "\n\n";
}

void KalmanFilter::printState(){
  State state = mm_->getState();
  std::cout << "State:\nPosition:\n" << state.position() << "\n\nVelocity:\n" << state.velocity() << "\n\nAngular Velocity:\n" << state.ang_vel() <<"\n\n";
}
