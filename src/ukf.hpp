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
     * Processes a measurement
     * @param measurement The latest measurement data of either radar or laser
     */
    void processMeasurement(const MeasurementPackage &measurement);

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
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void predict(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param measurement The measurement at k+1
     */
    void updateLidar(const MeasurementPackage &measurement);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param measurement The measurement at k+1
     */
    void updateRadar(const MeasurementPackage &measurement);

    /**
     * Initialize the state from the first measurement
     */
    void initialize(const MeasurementPackage &);

    /**
     * Generates Sigma points for the given parameters
     * @param x state vector
     * @param P process covariance matrix
     * @return the sigma points
     */
    MatrixXd generateSigmaPoints(const VectorXd &x, const MatrixXd &P);

    /**
     * Generates augmented Sigma points using std_a and std_yawdd
     * @param x state vector
     * @param P process covariance matrix
     * @param Q TODO
     * @return the augmented sigma points
     */
    MatrixXd generateAugmentedSigmaPoints(const VectorXd &x, const MatrixXd &P, const MatrixXd &Q);

    /**
     * Predict augmented sigma points
     * @param Xsig_aug the augmented sigma points
     * @param delta_t the time since the last update (in s)
     * @return the predicted sigma points
     */
    MatrixXd predictSigmaPoints(const MatrixXd &Xsig_aug, double delta_t);

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
     * Predicts the process covariance of the sigma points
     * @param Xsig_pred the predicted sigma points
     * @param x the predicted mean
     * @param weights the weights
     * @return the predicted process covariance
     */
    MatrixXd predictProcessCovariance(const MatrixXd &Xsig_pred, const VectorXd &x,
                                      const VectorXd &weights);


    /**
     * Converter function to convert sigma points to measurement space
     */
    using SigmaConverter = std::function<VectorXd(const VectorXd &)>;

    /**
     * Predicts the state in the measurement space
     * @param n_z the dimension of the measurement
     * @param R the noise covariance matrix to use
     * @param Zsig output parameter for the matrix for sigma points in measurement space
     * @param z_pred output parameter for the mean predicted measurement
     * @param S output parameter for the innovation covariance matrix S
     * @param converter the converter to use to convert the predicted sigma points to the measurement space
     */
    void predictMeasurement(int n_z, const MatrixXd &R, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S,
                            const SigmaConverter &converter);

    /**
     *
     * @param z measurement vector
     * @param Zsig matrix for sigma points in measurement space
     * @param z_pred mean predicted measurement
     * @param S Innovation covariance matrix S
     */
    void updateMeasurement(const VectorXd &z, const MatrixXd &Zsig, const VectorXd &z_pred, const MatrixXd &S);

private:
    // Provided constants //

    // Laser measurement noise standard deviation position1 in m
    const double std_laspx_{0.15};

    // Laser measurement noise standard deviation position2 in m
    const double std_laspy_{0.15};

    // Radar measurement noise standard deviation radius in m
    const double std_radr_{0.3};

    // Radar measurement noise standard deviation angle in rad
    const double std_radphi_{0.03};

    // Radar measurement noise standard deviation radius change in m/s
    const double std_radrd_{0.3};

    // Tweakable constants //

    // Process noise standard deviation longitudinal acceleration in m/s^2
//    const double std_a_{30};
    const double std_a_{2};
    //TODO

    // Process noise standard deviation yaw acceleration in rad/s^2
//    const double std_yawdd_{30};
    const double std_yawdd_{2};
    //TODO

    // if this is false, laser measurements will be ignored (except for init)
    const bool use_laser_{true};

    // if this is false, radar measurements will be ignored (except for init)
    const bool use_radar_{true};

    // State dimension
    const int n_x_{5};

    // Augmented state dimension
    const int n_aug_{7};

    // State //

    // initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_{false};

    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    // state covariance matrix
    MatrixXd P_;

    // predicted sigma points matrix
    MatrixXd Xsig_pred_;

    // time when the state is true, in us
    long long time_us_{0};

    // Weights of sigma points
    VectorXd weights_;

    // Process noise covariance matrix
    MatrixXd Q_;

    // Radar measurement covariance matrix
    MatrixXd R_radar_;

    // Radar measurement covariance matrix
    MatrixXd R_laser_;
};
