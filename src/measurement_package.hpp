#pragma once

#include <Eigen/Dense>

#include <vector>

class MeasurementPackage {
public:
    enum SensorType { LASER, RADAR };

    using Timestamp = long long;

    MeasurementPackage() = default;

    MeasurementPackage(SensorType type, Timestamp timestamp, std::vector<double> measurements)
        : sensor_type_(type), timestamp_(timestamp) {
        raw_measurements_ = Eigen::Map<Eigen::VectorXd>(measurements.data(), measurements.size());
    }

    SensorType sensor_type_;
    Timestamp timestamp_;
    Eigen::VectorXd raw_measurements_;
};
