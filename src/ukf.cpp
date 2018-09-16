#include "ukf.hpp"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30; // TODO

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30; // TODO

    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

    /**
    TODO:

    Complete the initialization. See ukf.h for other member properties.

    Hint: one or more values initialized above might be wildly off...
    */
}

UKF::~UKF() = default;

MatrixXd UKF::generateSigmaPoints(const MatrixXd &x, const MatrixXd &P) {

    // set state dimension
    auto n_x = x.rows();

    // define spreading parameter
    double lambda = 3 - n_x;

    // create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

    // calculate square root of P
    MatrixXd A = P.llt().matrixL();

    // Calculate Sigma points

    // col 0 -> xk|k
    Xsig.col(0) << x;

    // Other points
    for (size_t n = 0; n < n_x; n++) {
        Xsig.col(1 + n) << x + sqrt(lambda + n_x) * A.col(n);
        Xsig.col(1 + n_x + n) << x - sqrt(lambda + n_x) * A.col(n);
    }

    return Xsig;
}

MatrixXd UKF::generateAugmentedSigmaPoints(const MatrixXd &x, const MatrixXd &P, double std_a, double std_yawdd) {

    // set state dimension
    auto n_x = x.rows();

    //set augmented dimension
    auto n_aug = n_x + 2;

    //define spreading parameter
    double lambda = 3 - n_aug;

    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

    //create augmented mean state
    x_aug << x, 0, 0;

    //create augmented covariance matrix
    MatrixXd Q(2, 2);
    Q << std_a * std_a, 0,
            0, std_yawdd * std_yawdd;
    P_aug.block(0, 0, n_x, n_x) << P;
    P_aug.block(n_x, n_x, 2, 2) << Q;

    //create square root matrix
    MatrixXd A = P_aug.llt().matrixL();

    //create augmented sigma points
    // col 0 -> xk|k
    Xsig_aug.col(0) << x_aug;

    // Other points
    for (size_t n = 0; n < n_aug; n++) {
        Xsig_aug.col(1 + n) << x_aug + sqrt(lambda + n_aug) * A.col(n);
        Xsig_aug.col(1 + n_aug + n) << x_aug - sqrt(lambda + n_aug) * A.col(n);
    }

    return Xsig_aug;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
    TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */
}
