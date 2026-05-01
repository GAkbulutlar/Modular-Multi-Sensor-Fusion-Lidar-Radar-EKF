#include "tools.h"

double Tools::NormalizeAngle(double angle) {
    /**
     * Normalize angle to [-pi, pi] range
     * This is critical for Radar measurements where bearing angles
     * must be compared consistently
     */
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

Eigen::MatrixXd Tools::CalculateJacobianRadar(const Eigen::VectorXd& x) {
    /**
     * Calculate Jacobian of Radar measurement model h(x) = [rho, phi, rho_dot]
     * 
     * h(x) = [sqrt(px^2 + py^2),
     *         atan2(py, px),
     *         (px*vx + py*vy) / sqrt(px^2 + py^2)]
     * 
     * J = dh/dx is 3x4
     */
    Eigen::MatrixXd Hj(3, 4);
    
    double px = x(0);
    double py = x(1);
    double vx = x(2);
    double vy = x(3);
    
    double px2_py2 = px * px + py * py;
    double sqrt_px2_py2 = std::sqrt(px2_py2);
    double px2_py2_1p5 = px2_py2 * sqrt_px2_py2;
    
    // Handle division by zero
    if (px2_py2 < 1e-6) {
        std::cerr << "Warning: Near-zero distance in Jacobian calculation" << std::endl;
        Hj.setZero();
        return Hj;
    }
    
    // Jacobian of rho
    Hj(0, 0) = px / sqrt_px2_py2;
    Hj(0, 1) = py / sqrt_px2_py2;
    Hj(0, 2) = 0.0;
    Hj(0, 3) = 0.0;
    
    // Jacobian of phi
    Hj(1, 0) = -py / px2_py2;
    Hj(1, 1) = px / px2_py2;
    Hj(1, 2) = 0.0;
    Hj(1, 3) = 0.0;
    
    // Jacobian of rho_dot
    Hj(2, 0) = (py * (px * vy - py * vx)) / px2_py2_1p5;
    Hj(2, 1) = (px * (py * vx - px * vy)) / px2_py2_1p5;
    Hj(2, 2) = px / sqrt_px2_py2;
    Hj(2, 3) = py / sqrt_px2_py2;
    
    return Hj;
}

double Tools::ComputeNIS(const Eigen::VectorXd& innovation, 
                        const Eigen::MatrixXd& S) {
    /**
     * Compute Normalized Innovation Squared (NIS)
     * NIS = innovation^T * S^-1 * innovation
     * 
     * For consistency check:
     * - For Lidar (2D): ~95% of NIS values should be < 5.991 (chi-squared, df=2)
     * - For Radar (3D): ~95% of NIS values should be < 7.815 (chi-squared, df=3)
     */
    return (innovation.transpose() * S.inverse() * innovation)(0, 0);
}

Eigen::VectorXd Tools::PolarToCartesian(double rho, double phi, double rho_dot) {
    /**
     * Convert Radar polar coordinates to Cartesian
     * 
     * rho: range (distance)
     * phi: bearing angle
     * rho_dot: range rate (radial velocity)
     * 
     * Returns initial state [px, py, vx, vy]
     * Initial velocities are estimated from range rate and bearing
     */
    Eigen::VectorXd state(4);
    
    double px = rho * std::cos(phi);
    double py = rho * std::sin(phi);
    
    // Radial velocity only provides information along bearing direction
    // Without additional information, we estimate velocity along bearing
    double vx = rho_dot * std::cos(phi);
    double vy = rho_dot * std::sin(phi);
    
    state << px, py, vx, vy;
    return state;
}

Eigen::VectorXd Tools::CartesianToPolar(const Eigen::VectorXd& x) {
    /**
     * Convert Cartesian state to Radar polar coordinates
     * 
     * State: [px, py, vx, vy]
     * 
     * Returns: [rho, phi, rho_dot]
     * rho: sqrt(px^2 + py^2)
     * phi: atan2(py, px)
     * rho_dot: (px*vx + py*vy) / rho
     */
    Eigen::VectorXd z(3);
    
    double px = x(0);
    double py = x(1);
    double vx = x(2);
    double vy = x(3);
    
    double rho = std::sqrt(px * px + py * py);
    double phi = std::atan2(py, px);
    double rho_dot = 0.0;
    
    if (rho > 1e-6) {
        rho_dot = (px * vx + py * vy) / rho;
    }
    
    z << rho, phi, rho_dot;
    return z;
}
