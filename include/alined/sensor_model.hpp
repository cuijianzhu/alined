#pragma once

class SensorModel{

public:

  void updateState();

  void updateCovariance();

  void setCovariance(const Eigen::MatrixXd &cov);


private:


protected:

  Eigen::MatrixXd covariance_;

  Eigen::MatrixXd jacobian_;




};
