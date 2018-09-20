#include "ukf.hpp"
#include <coordinates.hpp>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

    // initial state vector
    x_ = VectorXd(n_x_);

    // initial covariance matrix
    P_ = MatrixXd::Identity(n_x_, n_x_);

    // Process noise covariance matrix
    Q_ = MatrixXd(2, 2);
    Q_ << std_a_ * std_a_, 0,
            0, std_yawdd_ * std_yawdd_;

    // Radar measurement covariance matrix
    R_radar_ = MatrixXd(3, 3);
    R_radar_.row(0) << std_radr_ * std_radr_, 0, 0;
    R_radar_.row(1) << 0, std_radphi_ * std_radphi_, 0;
    R_radar_.row(2) << 0, 0, std_radrd_ * std_radrd_;

    // Laser measurement covariance matrix
    R_laser_ = MatrixXd(2, 2);
    R_laser_.row(0) << std_laspx_ * std_laspx_, 0;
    R_laser_.row(1) << 0, std_laspy_ * std_laspy_;

    // Laser measurement matrix
    H_laser_ = MatrixXd(2, n_x_);
    H_laser_.row(0) << 1, 0, 0, 0, 0;
    H_laser_.row(1) << 0, 1, 0, 0, 0;
}

UKF::~UKF() {
    std::cout << "=== RADAR NIS===" << std::endl;
    std::cout << radarCSV << std::endl << std::endl;

    std::cout << "=== LASER NIS===" << std::endl;
    std::cout << laserCSV << std::endl << std::endl;
};

MatrixXd UKF::generateSigmaPoints(const VectorXd &x, const MatrixXd &P) {

    // define spreading parameter
    const double lambda = 3 - n_x_;

    // create sigma point matrix
    MatrixXd Xsig(n_x_, 2 * n_x_ + 1);

    // calculate square root of P
    MatrixXd A = P.llt().matrixL();

    // Calculate Sigma points

    // col 0 -> xk|k
    Xsig.col(0) << x;

    // Other points
    for (size_t n = 0; n < n_x_; n++) {
        Xsig.col(1 + n) << x + sqrt(lambda + n_x_) * A.col(n);
        Xsig.col(1 + n_x_ + n) << x - sqrt(lambda + n_x_) * A.col(n);
    }

    return Xsig;
}

MatrixXd UKF::generateAugmentedSigmaPoints(const VectorXd &x, const MatrixXd &P, const MatrixXd &Q) {
    //define spreading parameter
    const double lambda = 3 - n_aug_;

    //create augmented mean vector
    VectorXd x_aug(n_aug_);

    //create augmented state covariance
    MatrixXd P_aug(n_aug_, n_aug_);
    P_aug.fill(.0);

    //create sigma point matrix
    MatrixXd Xsig_aug(n_aug_, 2 * n_aug_ + 1);

    //create augmented mean state
    x_aug << x, 0, 0;

    //create augmented covariance matrix
    P_aug.block(0, 0, n_x_, n_x_) << P;
    P_aug.block(n_x_, n_x_, 2, 2) << Q;

    //create square root matrix
    MatrixXd A = P_aug.llt().matrixL();

    //create augmented sigma points
    // col 0 -> xk|k
    Xsig_aug.col(0) << x_aug;

    // Other points
    for (size_t n = 0; n < n_aug_; n++) {
        Xsig_aug.col(1 + n) << x_aug + sqrt(lambda + n_aug_) * A.col(n);
        Xsig_aug.col(1 + n_aug_ + n) << x_aug - sqrt(lambda + n_aug_) * A.col(n);
    }

    return Xsig_aug;
}

MatrixXd UKF::predictSigmaPoints(const MatrixXd &Xsig_aug, double delta_t) {
    // Number of sigma points
    const auto n_points = Xsig_aug.cols();

    // create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred(n_x_, n_points);

    for (size_t p = 0; p < n_points; p++) {
        auto sig = Xsig_aug.col(p);
        double px = sig(0);
        double py = sig(1);
        double v = sig(2);
        double yaw = sig(3);
        double yawd = sig(4);
        double nu_a = sig(5);
        double nu_yawdd = sig(6);

        VectorXd x(5);
        x << px, py, v, yaw, yawd;

        VectorXd a(5);

        if (abs(yawd) > 0.001) {
            a.row(0) << v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            a.row(1) << v / yawd * (-cos(yaw + yawd * delta_t) + cos(yaw));
        } else {
            a.row(0) << v * cos(yaw) * delta_t;
            a.row(1) << v * sin(yaw) * delta_t;
        }
        a.row(2) << 0;
        a.row(3) << yawd * delta_t;
        a.row(4) << 0;

        VectorXd b(5);
        b.row(0) << .5 * delta_t * delta_t * cos(yaw) * nu_a;
        b.row(1) << .5 * delta_t * delta_t * sin(yaw) * nu_a;
        b.row(2) << delta_t * nu_a;
        b.row(3) << .5 * delta_t * delta_t * nu_yawdd;
        b.row(4) << delta_t * nu_yawdd;

        Xsig_pred.col(p) << x + a + b;
    }

    return Xsig_pred;
}

VectorXd UKF::createWeights(int augmentedStateDimension) {
    const int n_sigma_points = 2 * augmentedStateDimension + 1;

    //create vector for weights
    VectorXd weights(n_sigma_points);

    //define spreading parameter
    double lambda = 3 - augmentedStateDimension;

    //set weights
    weights(0) = lambda / (lambda + augmentedStateDimension);
    for (int i = 1; i < n_sigma_points; i++) {
        weights(i) = 1 / (2 * (augmentedStateDimension + lambda));
    }

    return weights;
}

VectorXd UKF::predictMean(const MatrixXd &Xsig_pred, const VectorXd &weights) {
    //create vector for predicted state
    VectorXd x(Xsig_pred.rows());

    x.fill(.0);
    for (int i = 0; i < Xsig_pred.cols(); i++) {
        x += weights(i) * Xsig_pred.col(i);
    }

    return x;
}

MatrixXd UKF::predictProcessCovariance(const MatrixXd &Xsig_pred, const VectorXd &x, const VectorXd &weights) {
    //create covariance matrix for prediction
    MatrixXd P(Xsig_pred.rows(), Xsig_pred.rows());

    //predict state covariance matrix
    P.fill(.0);
    for (int i = 0; i < Xsig_pred.cols(); i++) {
        // Diff
        VectorXd diff = Xsig_pred.col(i) - x;

        // normalise angles
        Coordinates::normaliseAngle(diff(3));

        P += weights(i) * diff * diff.transpose();
    }

    return P;
}

void UKF::initialize(const MeasurementPackage &measurement) {
    if (MeasurementPackage::RADAR == measurement.sensor_type_) {
        double rho = measurement.raw_measurements_(0);
        double phi = measurement.raw_measurements_(1);
        double rho_dot = measurement.raw_measurements_(2);
        auto cartesian = CartesianCoordinate::fromPolar({rho, phi, rho_dot});
        x_ << cartesian.x, cartesian.y, cartesian.v(), 0, 0;
    } else if (MeasurementPackage::LASER == measurement.sensor_type_) {
        double px = measurement.raw_measurements_(0);
        double py = measurement.raw_measurements_(1);
        x_ << px, py, 0, 0, 0;
    } else {
        std::cerr << "Unknown measurement type" << std::endl;
        return;
    }

    // Initialize weights
    weights_ = createWeights(n_aug_);

    // Store initial timestamp for delta
    time_us_ = measurement.timestamp_;

    is_initialized_ = true;
}

void UKF::processMeasurement(const MeasurementPackage &measurement) {
    if (!is_initialized_) {
        initialize(measurement);
        return;
    }

    // Check if we're handling the measurement package
    if ((measurement.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) ||
        (measurement.sensor_type_ == MeasurementPackage::LASER && !use_laser_)) {
        std::cout << "Skipping unhandled measurement package" << std::endl;
        return;
    }

    // Calculate the time diff and store the ts for the next iteration
    double delta_t = (measurement.timestamp_ - time_us_) / 1000000.0;
    time_us_ = measurement.timestamp_;

    // Prediction
    predict(delta_t);

    // Update
    if (measurement.sensor_type_ == MeasurementPackage::RADAR) {
        updateRadar(measurement);
    } else if (measurement.sensor_type_ == MeasurementPackage::LASER) {
        updateLidar(measurement);
    } else {
        std::cout << "Skipping unknown measurement package" << std::endl;
    }
}

void UKF::predict(double delta_t) {
    MatrixXd sigmaPoints = generateAugmentedSigmaPoints(x_, P_, Q_);

    // Store as member to re-use in update
    Xsig_pred_ = predictSigmaPoints(sigmaPoints, delta_t);

    // Store state and co-variance
    x_ = predictMean(Xsig_pred_, weights_);
    P_ = predictProcessCovariance(Xsig_pred_, x_, weights_);
}

void UKF::updateLidar(const MeasurementPackage &measurement) {
    // For laser measurements we can do a linear update
    laserCSV.newRow() << linearUpdate(measurement.raw_measurements_, H_laser_, R_laser_);
}

void UKF::updateRadar(const MeasurementPackage &measurement) {

    // Measurement prediction //

    // set measurement dimension, radar can measure r, phi, and r_dot
    const int n_z = 3;

    auto sigmaConverter = [&n_z](const VectorXd &sigma) {
        // extract values for better readability
        double p_x = sigma(0);
        double p_y = sigma(1);
        double v = sigma(2);
        double yaw = sigma(3);

        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;

        // measurement model
        VectorXd converted(n_z);
        converted(0) = sqrt(p_x * p_x + p_y * p_y);                               //r
        converted(1) = atan2(p_y, p_x);                                           //phi
        converted(2) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);       //r_dot
        return converted;
    };

    MatrixXd Zsig;
    VectorXd z_pred;
    MatrixXd S;

    predictMeasurement(n_z, R_radar_, Zsig, z_pred, S, sigmaConverter);


    // Measurement update //

    // create vector for incoming radar measurement
    VectorXd z(n_z);

    double rho = measurement.raw_measurements_(0);
    double phi = measurement.raw_measurements_(1);
    double rho_dot = measurement.raw_measurements_(2);

    z << rho, phi, rho_dot;

    radarCSV.newRow() << updateMeasurement(z, Zsig, z_pred, S);
}

void UKF::predictMeasurement(int n_z, const MatrixXd &R, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S,
                             const SigmaConverter &converter) {
    // create matrix for sigma points in measurement space
    Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    Zsig.fill(0.0);

    // transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // measurement model
        const VectorXd &col = Xsig_pred_.col(i);
        Zsig.col(i) << converter(col);
    }

    // mean predicted measurement
    z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    // innovation covariance matrix S
    S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //r esidual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        Coordinates::normaliseAngle(z_diff(1));

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    S = S + R;
}

double UKF::updateMeasurement(const VectorXd &z, const MatrixXd &Zsig, const VectorXd &z_pred, const MatrixXd &S) {
    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, z.rows());
    Tc.fill(0.0);

    // calculate cross correlation matrix
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        Coordinates::normaliseAngle(z_diff(1));

        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        Coordinates::normaliseAngle(x_diff(3));

        Tc += weights_(i) * x_diff * z_diff.transpose();

    }

    // residual
    VectorXd z_diff = z - z_pred;
    Coordinates::normaliseAngle(z_diff(1));

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // update state mean and covariance matrix
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();

    // return NIS
    return z_diff.transpose() * S.inverse() * z_diff;
}

double UKF::linearUpdate(const VectorXd &z, const MatrixXd &H, const MatrixXd &R) {
    MatrixXd z_pred = H * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H.transpose();
    MatrixXd S = H * P_ * Ht + R;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    // new estimate
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * H) * P_;

    // return NIS
    return y.transpose() * S.inverse() * y;
}
