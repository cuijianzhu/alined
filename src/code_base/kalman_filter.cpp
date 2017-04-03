#include "alined/kalman_filter.hpp"

KalmanFilter::KalmanFilter(int64_t config){

}

void KalmanFilter::setMotionModel(MotionModel *mm){

  mm_ = mm;

  model_set_ = true;
}

void KalmanFilter::setInitialState(const MotionModel::State& state){

  assert(model_set_);

  mm_->setInitialState(state);

}

void KalmanFilter::setCovarianceMM(const Eigen::MatrixXd& cov_mm){

  assert(model_set_);

  mm->setCovariance(cov_mm);

}

void KalmanFilter::setCovarianceSM(const Eigen::MatrixXd& cov_sm, const u_char &sensor_id){

  assert(sensor_set_);

  sm[sensor_id]->setCovariance(cov_sm);

}

void KalmanFilter::pushSetSensorModel(SensorModel *sm){

  sm_.push_back(sm);

  sensor_set_ = true;
}

void KalmanFilter::predict(int64_t dt){

  assert(model_set_);

  mm->propagateState(dt);
  mm->propagateCovariance(dt);

}

void KalmanFilter::update(const u_char &sensor_id){

  assert(sensor_set_);

  sm[sensor_id]->updateState();
  sm[sensor_id]->updateCovariance();

}
