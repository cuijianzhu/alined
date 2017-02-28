#pragma once

#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/SVD"


class Alined{
public:

  enum DLT_METHOD_{LINE_DLT,COMBINED_LINES} method_;
  enum SOLVER_{LEAST_SQUARES,LEVENBERG_MARQUARDT} solver_;
  enum ITERATIVE_{NO_REFINEMENT,USE_ITERATIVE_REFINEMENT} iterative_;

  Alined(DLT_METHOD_ = COMBINED_LINES, ITERATIVE_ = NO_REFINEMENT);
  ~Alined();


  /*!
   * \brief Camera pose from line correspondences using DLT-Combined-Lines method
   *        Pose Estimation from Line Correspondences using Direct Linear Transformation" by
   *        Pribyl, B., Zemcik, P. and Cadik, M.
   * \param x_c - 3x(2N) 2D line endpoints (Don't need to correspond to 3D line endpoint locations)
   * \param X_w - 4x(2N) 3D line endpoints
   * \return Pose
   */
  Eigen::Matrix4d poseFromLines(Eigen::Matrix<double,3,Eigen::Dynamic> x_c, Eigen::Matrix<double,4,Eigen::Dynamic> X_w);

  /*!
   * \brief Camera pose using the iterative approach by Kumar and Hanson. This method needs a good prior.
   * \param x_c - 3x(2N) 2D line endpoints (Don't need to correspond to 3D line endpoint locations)
   * \param X_w - 4x(2N) 3D line endpoints
   * \return Pose
   */
  Eigen::Matrix4d poseFromLinesIterative(Eigen::Matrix4d pose, Eigen::Matrix<double,3,Eigen::Dynamic> x_c, Eigen::Matrix<double,4,Eigen::Dynamic> X_w, SOLVER_ solver = LEAST_SQUARES);



private:

  /*!
   * \brief Create Plucker Line from two points only
   * \param X - 4x(2N) homogeneous point pair
   * \return L - 6x1 Plucker Line
   */
  Eigen::Matrix<double,6,Eigen::Dynamic> createPluckerLines(const Eigen::Matrix<double,4,Eigen::Dynamic> &X);

  /*!
   * \brief Calculate the Kronecker product with matrices m1 and m2
   * \param m1 - matrix
   * \param m2 - matrix
   * \return Matrix
   */
  Eigen::MatrixXd kron(Eigen::MatrixXd m1, Eigen::MatrixXd m2);

  /*!
   * \brief Create a skew-symmetric matrix from a vector
   * \param vec - 3x1 vector
   * \return skew 3x3 matrix
   */
  inline Eigen::Matrix3d skew(const Eigen::Vector3d& vec);

  /*!
   * \brief Create a vector from a skew-symmetric matrix
   * \param skew - 3x3 matrix
   * \return vec - 3x1 vector
   */
  inline Eigen::Vector3d unskew(const Eigen::Matrix3d& skew);

  /*!
   * \brief Iteratively find the correct pose using the R_and_T algorithm by Kumar and Hanson 1994
   * \param tf - Initial Pose
   * \param X - Point matrix
   * \param l_c - 2D line Matrix
   * \return Pose
   */
  Eigen::Matrix4d refineIteratively(const Eigen::Matrix4d &tf, Eigen::Matrix<double,4, Eigen::Dynamic> X_w, Eigen::Matrix<double, 3, Eigen::Dynamic> l_c, Eigen::Matrix<double, 1, Eigen::Dynamic> w);

  /*!
   * \brief Iteratively find the correct pose using the R_and_T algorithm by Kumar and Hanson 1994
   *        in a full Levenberg-Marquardt scheme. This reduces divergent behavior at close to singular situations.
   * \param tf - Initial Pose
   * \param X - Point matrix
   * \param l_c - 2D line Matrix
   * \return Pose
   */
  Eigen::Matrix4d levenbergMarquardt(const Eigen::Matrix4d &tf, Eigen::Matrix<double,4, Eigen::Dynamic> X_w, Eigen::Matrix<double, 3, Eigen::Dynamic> l_c, Eigen::Matrix<double, 1, Eigen::Dynamic> w);

  /*!
   * \brief Calculate the Huber loss function to penalize outliers in the nonlinear optimization
   * \param cost - incremental cost of measurement i
   * \return weight - weigth to be used in outlier rejection
   */
  double huberLoss(double cost, double scale, double order);

  /*!
   * \brief Calculate the Huber loss function to penalize outliers in the nonlinear optimization
   * \param cost - incremental cost of measurement i
   * \return weight - weigth to be used in outlier rejection
   */
  double cauchyLoss(double cost, double scale, double order);

};


