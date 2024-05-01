#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::ArrayXd;  // For element-wise operations

/**
 * Initialize Unscented Kálmán Filter (UKF)
 */
UKF::UKF()
{
  // If this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // If this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Initial state vector
  x_ = VectorXd(5);

  // Initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration, in m/s^2
  std_a_ = 2.;

  // Process noise standard deviation yaw acceleration, in rad/s^2
  std_yawdd_ = 2.;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1, in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2, in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius, in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle, in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change, in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values.
   */

  is_initialized_ = false;

  n_x_ = 5;
  n_aug_ = n_x_ + 2;  // [px, py, v, psi, psi_dot, std_a_, std_yawdd_]
  n_z_ = 3;

  /** 
   * [1] - Lesson 17: Augmentation Assignment 1, Augmented Kálmán Filters, Udacity Sensor Fusion
   * [2] - https://knowledge.udacity.com/questions/703758
   */
  lambda_ = 3 - n_aug_;  // As suggested in [1], [2] and instead of setting to (3 - n_x_)

  Xsig_aug_.resize(n_aug_, 2 * n_aug_ + 1);  // Custom addition

  Xsig_pred_.resize(n_x_, 2 * n_aug_ + 1);

  weights_.resize(2 * n_aug_ + 1);
  weights_.setConstant(0.5 / (lambda_ + n_aug_));  // 0.5 / (lambda_ + n_aug_)   
  weights_[0] *= (lambda_ / 0.5);  // lambda_ / (lambda_ + n_aug_)
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
  if (!is_initialized_)  // https://knowledge.udacity.com/questions/759764, 654359
  {
    switch (meas_package.sensor_type_)
    {
      case MeasurementPackage::SensorType::LASER:

        // px, py
        x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;

        /**
         * To pass RMSE tests for all combinations of single and pairs of cars as well, the last 3
         * diagonal elements of P_ are set to 1, 0.5, 0.25. If visualize_pcd = true, the RMSE thresholds
         * will still be breached for X because the LiDAR point cloud often does not capture the full
         * shape of the target objects.
         */
        P_.setZero();
        P_.diagonal() << pow(std_laspx_, 2), pow(std_laspy_, 2), 1, 0.5, 0.25;

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
  }

  // Elapsed time between previous and current observation
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
  // Generate UKF sigma points based on posterior state vector x_{k|k} and covariance matrix P_{k|k}
  AugmentedSigmaPoints();

  // Predict sigma points at k+1 via CTRV (Constant Turn Rate and Velocity Magnitude) process model
  SigmaPointsPrediction(delta_t);

  // Predict mean state vector x_{k+1|k} and covariance matrix P_{k+1|k} using sigma points
  PredictMeanAndCovariance();
}

void UKF::UpdateLidar(MeasurementPackage meas_package)  // Kálmán Filter [1]
{
  // Measurement matrix
  MatrixXd H_ = MatrixXd::Zero(2, n_x_);
  H_.leftCols(2) = Eigen::Matrix2d::Identity();

  // Measurement noise covariance matrix
  Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();
  R_.diagonal() << pow(std_laspx_, 2), pow(std_laspy_, 2);

  // Predicted measurement mean
  z_pred_ = H_ * x_;

  // Prediction error
  VectorXd y = meas_package.raw_measurements_ - z_pred_;

  // Predicted measurement covariance matrix
  S_ = H_ * P_ * H_.transpose() + R_;

  // Kálmán gain
  MatrixXd K = P_ * H_.transpose() * S_.inverse();

  x_ += K * y;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)  // Unscented Kálmán Filter
{
  // Predict measurement mean z_{k+1|k} and covariance matrix S_{k+1|k} using sigma points
  PredictRadarMeasurement();

  // Update mean state vector x_{k+1|k+1} and covariance matrix P_{k+1|k+1}
  UpdateState(meas_package);
}

/*********************************************************************************************************************
 * CUSTOM ADDITIONS
 *********************************************************************************************************************/

void UKF::AugmentedSigmaPoints()
{
  // Augmented mean state vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug.head(n_x_) = x_;  // Mean of noise components (last two elements) is zero

  // Process noise covariance matrix
  Eigen::DiagonalMatrix<double, 2> Q(pow(std_a_, 2), pow(std_yawdd_, 2));

  // Augmented state covariance matrix
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;

  MatrixXd A = P_aug.llt().matrixL();  // A * A.transpose() = P_aug

  // Augmented sigma points matrix
  Xsig_aug_.colwise() = x_aug;
  Xsig_aug_.block(0, 1, n_aug_, n_aug_) += sqrt(lambda_ + n_aug_) * A;
  Xsig_aug_.block(0, n_aug_ + 1, n_aug_, n_aug_) -= sqrt(lambda_ + n_aug_) * A;
}

void UKF::SigmaPointsPrediction(double& delta_t)
{
  // Predict sigma points (element-wise operations)
  ArrayXd px = Xsig_aug_.row(0);
  ArrayXd py = Xsig_aug_.row(1);
  ArrayXd v = Xsig_aug_.row(2);
  ArrayXd psi = Xsig_aug_.row(3);
  ArrayXd psi_dot = Xsig_aug_.row(4);
  ArrayXd nu_a = Xsig_aug_.row(5);
  ArrayXd nu_yawdd = Xsig_aug_.row(6);

  // State augmentation matrix
  MatrixXd state_aug = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  for (int i = 0; i < psi_dot.size(); ++i)
  {
    // Avoid division by zero
    if (fabs(psi_dot[i]) > 1e-3)  // Vehicle turns
    {
      state_aug(0, i) = v(i) / psi_dot(i) * (sin(psi(i) + psi_dot(i)*delta_t) - sin(psi(i)));
      state_aug(1, i) = v(i) / psi_dot(i) * (-cos(psi(i) + psi_dot(i)*delta_t) + cos(psi(i)));

    } else  // Vehicle drives straight
    {
      state_aug(0, i) = v(i) * cos(psi(i)) * delta_t;
      state_aug(1, i) = v(i) * sin(psi(i)) * delta_t;
    }
  }

  state_aug.row(3) = psi_dot * delta_t;

  // Noise matrix
  MatrixXd nu = MatrixXd(n_x_, 2 * n_aug_ + 1);
  double dt_squared = pow(delta_t, 2);
  nu.row(0) = 0.5 * dt_squared * cos(psi) * nu_a;
  nu.row(1) = 0.5 * dt_squared * sin(psi) * nu_a;
  nu.row(2) = delta_t * nu_a;
  nu.row(3) = 0.5 * dt_squared * nu_yawdd;
  nu.row(4) = delta_t * nu_yawdd;

  // Predicted sigma points
  Xsig_pred_.setZero();
  Xsig_pred_ = Xsig_aug_.topRows(n_x_) + state_aug + nu;
}

ArrayXd UKF::normalizeAnglesHelper(ArrayXd arr)  // Refactoring suggested by Udacity GPT
{
  return arr.unaryExpr([](double angle) { return fmod(angle + M_PI, 2.0 * M_PI) - M_PI; });
}

void UKF::PredictMeanAndCovariance()
{
  // Predicted state mean
  x_ = Xsig_pred_ * weights_;  // To invert the spread of the sigma points

  MatrixXd x_diff_ = Xsig_pred_.colwise() - x_;  // Includes angle subtraction

  // Normalization for psi required after angle subtraction
  x_diff_.row(3) = normalizeAnglesHelper(x_diff_.row(3));

  // Predicted state covariance matrix
  P_ = x_diff_ * weights_.asDiagonal() * x_diff_.transpose();
}

void UKF::PredictRadarMeasurement()
{
  ArrayXd px_pred = Xsig_pred_.row(0);  // Use ArrayXd for element-wise operations
  ArrayXd py_pred = Xsig_pred_.row(1);
  ArrayXd v_pred = Xsig_pred_.row(2);
  ArrayXd psi_pred = Xsig_pred_.row(3);

  x_diff_ = Xsig_pred_.colwise() - x_;
  x_diff_.row(3) = normalizeAnglesHelper(x_diff_.row(3));

  MatrixXd Zsig_pred = MatrixXd(n_z_, 2 * n_aug_ + 1);  // [rho (m), phi (rad), rho_dot (m/s)]
  ArrayXd rho_pred = (px_pred.square() + py_pred.square()).sqrt();
  Zsig_pred.row(0) = rho_pred;
  Zsig_pred.row(1) = py_pred.binaryExpr(px_pred, [](double py, double px) { return atan2(py, px); });
  Zsig_pred.row(2) = (px_pred * cos(psi_pred) * v_pred + py_pred * sin(psi_pred) * v_pred) / rho_pred;

  // Predicted measurement mean
  z_pred_ = Zsig_pred * weights_;

  z_diff_ = Zsig_pred.colwise() - z_pred_;  // Includes angle subtraction
  z_diff_.row(1) = normalizeAnglesHelper(z_diff_.row(1));

  // Predicted measurement noise covariance matrix
  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  R_.diagonal() << pow(std_radr_, 2), pow(std_radphi_, 2), pow(std_radrd_, 2);

  // Predicted measurement covariance matrix
  S_ = z_diff_ * weights_.asDiagonal() * z_diff_.transpose() + R_;
}

void UKF::UpdateState(MeasurementPackage meas_package)
{
  // Cross-correlation between sigma points in predicted and measurement spaces
  MatrixXd Tc = x_diff_ * weights_.asDiagonal() * z_diff_.transpose();

  // Kálmán gain
  MatrixXd K = Tc * S_.inverse();  

  VectorXd z_difference = (meas_package.raw_measurements_ - z_pred_);  // Includes angle subtraction
  z_difference.row(1) = normalizeAnglesHelper(z_difference.row(1));

  x_ += K * z_difference;
  P_ -= K * S_ * K.transpose();
}