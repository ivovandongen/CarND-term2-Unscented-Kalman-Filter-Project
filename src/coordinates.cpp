#include "coordinates.hpp"

#include <algorithm>
#include <cmath>

CartesianCoordinate CartesianCoordinate::fromPolar(PolarCoordinate polar) {
    static const double threshold = 0.0001;
    return {std::max<double>(polar.rho * cos(polar.phi), threshold),
            std::max<double>(polar.rho * sin(polar.phi), threshold),
            std::max<double>(polar.rho_dot * cos(polar.phi), threshold),
            std::max<double>(polar.rho_dot * sin(polar.phi), threshold)};
}

double CartesianCoordinate::v() const {
    return sqrt(vx * vx + vy + vy);
}

PolarCoordinate PolarCoordinate::fromCartesian(CartesianCoordinate ct) {
    double rho = sqrt(ct.x * ct.x + ct.y * ct.y);
    double phi = atan2(ct.y, ct.x);
    double rho_dot = (ct.x * ct.vx + ct.y * ct.vy) / rho;

    // Normalise phi
    Coordinates::normaliseAngle(phi);

    return {rho, phi, rho_dot};
}

void Coordinates::normaliseAngle(double &angle) {
    static const double TWO_PI = 2 * M_PI;
    while (abs(angle) > M_PI) {
        angle = (angle < -M_PI) ? angle + TWO_PI : angle - TWO_PI;
    }
}