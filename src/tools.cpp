#include <tools.hpp>

#include <iostream>
#include <vector>

Tools::Tools() = default;

Tools::~Tools() = default;

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                     const std::vector<Eigen::VectorXd> &ground_truth) {
    Eigen::VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // Sanity checks
    if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        std::cout << "Estimation or ground truth vector size is incorrect";
        return rmse;
    }

    // accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i) {
        Eigen::VectorXd residuals = (estimations[i] - ground_truth[i]);
        residuals = residuals.array() * residuals.array();
        rmse += residuals;
    }

    rmse /= estimations.size();
    return rmse.array().sqrt();
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd &x_state) {
    Eigen::MatrixXd Hj(3, 4);

    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    double c1 = px * px + py * py;
    double c2 = sqrt(c1);

    // check division by zero
    if (c1 < 0.0001) {
        std::cout << "Jacobian - error division by zero" << std::endl;
        return Hj;
    }

    // compute the Jacobian matrix
    Hj.row(0) << px / c2, py / c2, 0, 0;
    Hj.row(1) << -py / (c2 * c2), px / (c2 * c2), 0, 0;
    Hj.row(2) << py * (vx * py - vy * px) / (c2 * c2 * c2), px * (vy * px - vx * py) / (c2 * c2 * c2), px / c2, py / c2;

    return Hj;
}
