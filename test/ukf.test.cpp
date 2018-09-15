#include <test.hpp>

#include <ukf.hpp>

TEST(UKF, GenerateSigmaPoints) {
    UKF ukf;

    int n_x = 5;

    // State
    VectorXd x = VectorXd(n_x);
    x << 5.7441,
            1.3800,
            2.2049,
            0.5015,
            0.3528;

    // Covariance matrix
    MatrixXd P = MatrixXd(n_x, n_x);
    P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
            -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
            0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
            -0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
            -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

    // Generate sigma points
    MatrixXd sigmaPoints = ukf.generateSigmaPoints(x, P);

    // Expected
    MatrixXd Xsig_expected(5, 2 * n_x + 1);
    Xsig_expected.row(0) << 5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441;
    Xsig_expected.row(1) << 1.38, 1.34566, 1.52806, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38;
    Xsig_expected.row(2)
            << 2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049;
    Xsig_expected.row(3)
            << 0.5015, 0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.55961, 0.371114, 0.486077, 0.407773, 0.5015;
    Xsig_expected.row(4)
            << 0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879;


    ASSERT_TRUE(Xsig_expected.isApprox(sigmaPoints, 0.0001));
}