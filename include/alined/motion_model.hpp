#pragma once
#include "Eigen/Eigen"
#include <functional>
#include <iostream>

class MotionModel{

public:

  MotionModel();
  MotionModel(const MotionModel&) = delete;

  struct State{
  protected:
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Quaterniond rotation_;
    Eigen::Vector3d angular_velocity_;

  public:

    Eigen::Vector3d& position(){return position_;}
    Eigen::Vector3d& velocity(){return velocity_;}
    Eigen::Quaterniond& rotation(){return rotation_;}
    Eigen::Vector3d& ang_vel(){return angular_velocity_;}
    double& x(){return position_(0,0);}
    double& y(){return position_(1,0);}
    double& z(){return position_(2,0);}

  };

  State& getState();
  void setState();

  void setInitialState(State init_state);
  void setCovariance(const Eigen::MatrixXd &cov);

  void propagateState(int64_t dt);
  void propagateCovariance(int64_t dt);



private:

protected:

 Eigen::MatrixXd covariance_;

 Eigen::MatrixXd jacobian_;

 void constant_velocity_model(int64_t dt);

 std::function<void(int64_t dt)> f;

 State state_, init_state_;


};
