#pragma once
#include "Eigen/Eigen"

namespace ekf{

/*!
 * \brief state TODO: create abstraction
 */
struct State{
protected:
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Quaterniond rotation_;
  Eigen::Quaterniond innovation_;
  Eigen::Vector3d angular_velocity_;

public:

  Eigen::Vector3d& position(){return position_;}
  Eigen::Vector3d& velocity(){return velocity_;}
  Eigen::Quaterniond& rotation(){return rotation_;}
  Eigen::Quaterniond& innovation(){return innovation_;}
  Eigen::Vector3d& ang_vel(){return angular_velocity_;}
  double& x(){return position_(0,0);}
  double& y(){return position_(1,0);}
  double& z(){return position_(2,0);}

};
}
