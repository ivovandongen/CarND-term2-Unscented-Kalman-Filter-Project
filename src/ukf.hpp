#pragma once

#include <Eigen/Dense>
#include <measurement_package.hpp>

#include <fstream>
#include <string>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage meas_package);

    /**
     * @return the state vector x
     */
    const Eigen::VectorXd &getX() const { return x_; }

    /**
     * @return the process covariance matrix P
     */
    const Eigen::MatrixXd &getP() const { return P_; }

protected:
    /**
     * Generates Sigma points for the given parameters
     * @param x state vector
     * @param P process covariance matrix
     * @return the sigma points
     */
    MatrixXd generateSigmaPoints(const MatrixXd &x, const MatrixXd &P);

    /**
     * Generates augmented Sigma points using std_a and std_yawdd
     * @param x state vector
     * @param P process covariance matrix
     * @param std_a Process noise standard deviation longitudinal acceleration in m/s^2
     * @param std_yawdd Process noise standard deviation yaw acceleration in rad/s^2
     * @return the augmented sigma points
     */
    MatrixXd generateAugmentedSigmaPoints(const MatrixXd &x, const MatrixXd &P, double std_a, double std_yawdd);

    /**
     * Predict augmented sigma points
     * @param Xsig_aug the augmented sigma points
     * @param stateDimension the length of the state vector x
     * @param delta_t the time since the last update (in s)
     * @return the predicted sigma points
     */
    MatrixXd predictSigmaPoints(const MatrixXd &Xsig_aug, int stateDimension, double delta_t);

    /**
     * Create weights for state and process covariance prediction
     * @param augmentedStateDimension the size of the augmented state mastrix
     * @return the weights
     */
    VectorXd createWeights(int augmentedStateDimension);

    /**
     * Predict the mean of the sigma points
     * @param Xsig_pred the predicted sigma points
     * @param weights the weights
     * @return the predicted mean vector x
     */
    VectorXd predictMean(const MatrixXd &Xsig_pred, const VectorXd &weights);

    /**
     * Predicts the
     * @param Xsig_pred the predicted sigma points
     * @param x the predicted mean
     * @param weights the weights
     * @return the predicted process covariance
     */
    MatrixXd predictProcessCovariance(const MatrixXd &Xsig_pred, const MatrixXd &x,
                                      const VectorXd &weights);

private:
    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;

    ///* time when the state is true, in us
    long long time_us_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Sigma point spreading parameter
    double lambda_;
};
