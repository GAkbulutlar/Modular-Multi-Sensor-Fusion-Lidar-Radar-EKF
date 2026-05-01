#ifndef TOOLS_H
#define TOOLS_H
#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

/**
 * @class Tools
 * @brief Utility functions for EKF computations
 */
class Tools {
public:
    /**
     * @brief Normalize angle to [-pi, pi]
     * @param angle The angle in radians to normalize
     * @return The normalized angle in [-pi, pi]
     */
    static double NormalizeAngle(double angle);

    /**
     * @brief Calculate Jacobian matrix for Radar measurement model
     * @param x State vector [px, py, vx, vy]
     * @return 3x4 Jacobian matrix
     */
    static Eigen::MatrixXd CalculateJacobianRadar(const Eigen::VectorXd& x);

    /**
     * @brief Compute Normalized Innovation Squared (NIS) for Lidar
     * @param innovation Innovation vector (z - z_pred)
     * @param S Innovation covariance matrix (H*P*H^T + R)
     * @return NIS value
     */
    static double ComputeNIS(const Eigen::VectorXd& innovation, 
                            const Eigen::MatrixXd& S);

    /**
     * @brief Convert Radar polar coordinates to Cartesian
     * @param rho Range
     * @param phi Bearing
     * @param rho_dot Range rate
     * @return Vector [x, y, vx, vy] in Cartesian coordinates
     */
    static Eigen::VectorXd PolarToCartesian(double rho, double phi, double rho_dot);

    /**
     * @brief Convert Cartesian coordinates to Radar polar
     * @param x State vector [px, py, vx, vy]
     * @return Vector [rho, phi, rho_dot] in polar coordinates
     */
    static Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd& x);
};

#endif // TOOLS_H
