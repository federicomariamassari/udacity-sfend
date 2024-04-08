#include "ukf.h"
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values.
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  is_initialized_ = false;

  n_x_ = 5;
  n_aug_ = n_x_ + 2;  // [px, py, v, psi, psi_dot, std_a_, std_yawdd_]

  n_z_ = 3;

  lambda_ = 3 - n_x_;

  weights_.resize(2 * n_aug_ + 1);
  weights_[0] = lambda_ / (lambda_ + n_aug_);
  weights_.segment(1, weights_.size()-1).setConstant(0.5 / (lambda_ + n_aug_));
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (!is_initialized_)  // https://knowledge.udacity.com/questions/759764, 654359
  {
    switch (meas_package.sensor_type_)
    {
      case MeasurementPackage::SensorType::LASER:

        // px, py
        x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;

        P_.setZero();
        P_.diagonal() << pow(std_laspx_, 2), pow(std_laspy_, 2), 1, 1, 1;

        break;

      case MeasurementPackage::SensorType::RADAR:

        const double rho_ = meas_package.raw_measurements_[0];
        const double phi_ = meas_package.raw_measurements_[1];
        const double rho_dot_ = meas_package.raw_measurements_[2];

        const double px_ = cos(phi_) * rho_;  // adjacent = cos(phi_) * hypothenuse
        const double py_ = sin(phi_) * rho_;  // opposite = sin(phi_) * hypothenuse
        const double v_ = rho_dot_;

        const double vx_ = cos(phi_) * v_;
        const double vy_ = sin(phi_) * v_;

        // No information on yaw angle and yaw rate from radar measurement alone (source: Udacity GPT)
        const double psi_ = 0;
        const double psi_dot_ = 0;

        x_ << px_, py_, v_, psi_, psi_dot_;

        P_.setZero();
        P_.diagonal() << pow(std_radr_, 2), pow(std_radr_, 2), pow(std_radrd_, 2), pow(std_radphi_, 2), 
          pow(std_radphi_, 2);

        break;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    
    return;  // No further action required
  }

  double dt = (meas_package.timestamp_ - time_us_) * 1e-6;  // Microseconds to seconds
  
  time_us_ = meas_package.timestamp_;
  
  Prediction(dt);

  switch (meas_package.sensor_type_)
  {
    case MeasurementPackage::SensorType::LASER:

      if (use_laser_)
        UpdateLidar(meas_package);

      break;

    case MeasurementPackage::SensorType::RADAR:

      if (use_radar_) 
        UpdateRadar(meas_package);

      break;
  }
}

void UKF::Prediction(double delta_t) 
{
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // Augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  VectorXd x_aug = VectorXd::Zero(n_aug_);  // Mean of noise components (last two elements) is zero
  x_aug.head(n_x_) = x_;

  // Process noise covariance matrix
  Eigen::DiagonalMatrix<double, 2> Q(pow(std_a_, 2), pow(std_yawdd_, 2));

  // Augmented state covariance matrix
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;

  MatrixXd A = P_aug.llt().matrixL();  // A * A.transpose() = P_aug

  Xsig_aug.colwise() = x_aug;
  Xsig_aug.block(0, 1, n_aug_, n_aug_) += sqrt(lambda_ + n_aug_) * A;
  Xsig_aug.block(0, n_aug_ + 1, n_aug_, n_aug_) -= sqrt(lambda_ + n_aug_) * A;

  // Predict sigma points
  VectorXd px = Xsig_aug.row(0);
  VectorXd py = Xsig_aug.row(1);
  VectorXd v = Xsig_aug.row(2);
  VectorXd psi = Xsig_aug.row(3);
  VectorXd psi_dot = Xsig_aug.row(4);
  VectorXd nu_a = Xsig_aug.row(5);
  VectorXd nu_yawdd = Xsig_aug.row(6);

  // State augmentation matrix
  MatrixXd state_aug = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  for (int i = 0; i < psi_dot.size(); ++i)
  {
    if (fabs(psi_dot[i]) > 1e-3)  // Avoid division by zero
    {
      state_aug(0, i) = v(i) / psi_dot(i) * (sin(psi(i) + psi_dot(i)*delta_t) - sin(psi(i)));
      state_aug(1, i) = v(i) / psi_dot(i) * (-cos(psi(i) + psi_dot(i)*delta_t) + cos(psi(i)));

    } else 
    {
      state_aug(0, i) = v(i) * cos(psi(i)) * delta_t;
      state_aug(1, i) = v(i) * sin(psi(i)) * delta_t;
    }
  }

  state_aug.row(3) = psi_dot.array() * delta_t;

  // Noise matrix
  MatrixXd nu = MatrixXd(n_x_, 2 * n_aug_ + 1);

  nu.row(0) = 0.5 * pow(delta_t, 2) * cos(psi.array()) * nu_a.array();
  nu.row(1) = 0.5 * pow(delta_t, 2) * sin(psi.array()) * nu_a.array();
  nu.row(2) = delta_t * nu_a.array();
  nu.row(3) = 0.5 * pow(delta_t, 2) * nu_yawdd.array();
  nu.row(4) = delta_t * nu_yawdd.array();

  // Predicted sigma points
  Xsig_pred_.setZero();

  Xsig_pred_ = Xsig_aug.topRows(n_x_) + state_aug + nu;

  // Predicted state mean
  x_ = Xsig_pred_ * weights_;

  MatrixXd x_diff = Xsig_pred_.colwise() - x_;

  // Predict state covariance matrix (refactoring suggested by Udacity GPT)
  x_diff.row(3) = x_diff.row(3).unaryExpr([](double angle)
    {
      return fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
    });

  P_ = x_diff * weights_.asDiagonal() * x_diff.transpose();
}

void UKF::UpdateLidar(MeasurementPackage meas_package) 
{
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // Lesson 13: Laser Measurements Part 3, Lidar and Radar Fusion with Kalman Filters in C++

  MatrixXd H_ = MatrixXd::Zero(2, n_x_);
  H_.leftCols(2) = Eigen::Matrix2d::Identity();

  // Measurement noise covariance matrix
  Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();
  R_.diagonal() << pow(std_laspx_, 2), pow(std_laspy_, 2);

  VectorXd z_pred = H_ * x_;
  VectorXd y = meas_package.raw_measurements_ - z_pred;

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ += K * y;
  
  long x_size = x_.size();
  MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) 
{
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  MatrixXd Zsig_pred = MatrixXd(n_z_, 2 * n_aug_ + 1);

  VectorXd px_pred = Xsig_pred_.row(0);
  VectorXd py_pred = Xsig_pred_.row(1);
  VectorXd v_pred = Xsig_pred_.row(2);
  VectorXd psi_pred = Xsig_pred_.row(3);

  Zsig_pred.row(0) = (px_pred.array().square() + py_pred.array().square()).sqrt();
  
  Zsig_pred.row(1) = py_pred.array().binaryExpr(px_pred.array(), [](double py, double px) { return atan2(py, px); });
  
  Zsig_pred.row(2) = (px_pred.array() * cos(psi_pred.array()) * v_pred.array() + py_pred.array() * 
    sin(psi_pred.array()) * v_pred.array()) / (px_pred.array().square() + py_pred.array().square()).sqrt();

  VectorXd z_pred = Zsig_pred * weights_;

  MatrixXd x_diff = Xsig_pred_.colwise() - x_;

  MatrixXd z_diff = Zsig_pred.colwise() - z_pred;

  // Predict state covariance matrix (refactoring suggested by Udacity GPT)
  x_diff.row(3) = x_diff.row(3).unaryExpr([](double angle)
    {
      return fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
    });

  z_diff.col(3) = z_diff.col(3).unaryExpr([](double angle)
    {
      return fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
    });

  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  R_.diagonal() << pow(std_radr_, 2), pow(std_radphi_, 2), pow(std_radrd_, 2);

  MatrixXd S = z_diff * weights_.asDiagonal() * z_diff.transpose() + R_;

  MatrixXd Tc = x_diff * weights_.asDiagonal() * z_diff.transpose();

  MatrixXd K = Tc * S.inverse();

  VectorXd z_difference = (meas_package.raw_measurements_ - z_pred);

  z_difference.unaryExpr([](double angle)
    {
      return fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
    });

  x_ += K * z_difference;
  P_ -= K * S * K.transpose();
}