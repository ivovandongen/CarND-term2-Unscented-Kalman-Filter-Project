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
