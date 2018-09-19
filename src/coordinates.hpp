#pragma once

class Coordinates {
public:
    static void normaliseAngle(double &angle);
};

class PolarCoordinate;

class CartesianCoordinate {
public:
    CartesianCoordinate(double x_, double y_, double vx_ = 0, double vy_ = 0) : x(x_), y(y_), vx(vx_), vy(vy_) {}

    double v() const;

    static CartesianCoordinate fromPolar(PolarCoordinate polar);

    const double x, y, vx, vy;
};

class PolarCoordinate {
public:
    PolarCoordinate(double rho_, double phi_, double rho_dot_ = 0) : rho(rho_), phi(phi_), rho_dot(rho_dot_) {}

    static PolarCoordinate fromCartesian(CartesianCoordinate cartesian);

    const double rho, phi, rho_dot;
};
