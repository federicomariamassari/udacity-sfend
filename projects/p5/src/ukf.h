#ifndef UKF_H
#define UKF_H

#include <iostream>

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF
{
  public:

    UKF();

    virtual ~UKF();

    /**
     * @brief ProcessMeasurement.
     * 
     * @param meas_package The latest measurement data of either radar or laser.
     */
    void ProcessMeasurement(MeasurementPackage meas_package);

    /**
     * @brief Prediction predicts sigma points, the state, and the state covariance matrix.
     *
     * @param delta_t Time between k and k+1 in seconds.
     */
    void Prediction(double delta_t);

    /**
     * @brief Updates the state and the state covariance matrix using a laser measurement.
     * 
     * @param meas_package The measurement at k+1.
     * 
     * Resources:
     * 
     * [1] - Lesson 13: Laser Measurements Part 3, Lidar and Radar Fusion with Kálmán Filters in C++
     */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
     * @brief Updates the state and the state covariance matrix using a radar measurement.
     * 
     * @param meas_package The measurement at k+1.
     */
    void UpdateRadar(MeasurementPackage meas_package);

    /** 
     * @brief Generate UKF sigma points based on posterior state vector x_{k|k} and covariance matrix P_{k|k}
     *   representing the distribution of the current state (custom addition).
     */
    void AugmentedSigmaPoints();

    /**
     * @brief Predict k+1 sigma points according to CTRV model (custom addition).
     * 
     * @param delta_t Current time step.
     */
    void SigmaPointsPrediction(double& delta_t);

    /**
     * @brief 
     * 
     * @param arr An array, usually a matrix row, of angles to normalize.
     */
    Eigen::ArrayXd normalizeAnglesHelper(Eigen::ArrayXd arr);

    /**
     * @brief Predict a priori mean state vector x_{k+1|k} and covariance matrix P_{k+1|k} using sigma points.
     */
    void PredictMeanAndCovariance();

    // Initially set to false, set to true in the first call of ProcessMeasurement
    bool is_initialized_;

    // If false, laser measurements will be ignored (except for initialisation)
    bool use_laser_;

    // If false, radar measurements will be ignored (except for initialisation)
    bool use_radar_;

    // State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x_;

    // State covariance matrix
    Eigen::MatrixXd P_;

    // Augmented sigma points matrix (custom addition)
    Eigen::MatrixXd Xsig_aug_;

    // Predicted sigma points matrix
    Eigen::MatrixXd Xsig_pred_;

    // Time when the state is true, in us (microseconds)
    long long time_us_;

    // Process noise standard deviation longitudinal acceleration, in m/s^2
    double std_a_;

    // Process noise standard deviation yaw acceleration, in rad/s^2
    double std_yawdd_;

    // Laser measurement noise standard deviation position1, in m
    double std_laspx_;

    // Laser measurement noise standard deviation position2, in m
    double std_laspy_;

    // Radar measurement noise standard deviation radius, in m
    double std_radr_;

    // Radar measurement noise standard deviation angle, in rad
    double std_radphi_;

    // Radar measurement noise standard deviation radius change, in m/s
    double std_radrd_;

    // Weights of the sigma points
    Eigen::VectorXd weights_;

    // State dimension
    int n_x_;

    // Augmented state dimension
    int n_aug_;

    // Radar measurement dimension
    int n_z_;

    // Sigma point spreading parameter
    double lambda_;
};

#endif /* UKF_H */