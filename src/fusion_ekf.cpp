#include "fusion_ekf.h"
#include "tools.h"
#include <iostream>
#include <iomanip>
#include <cmath>

FusionEKF::FusionEKF() 
    : previous_timestamp_(0), is_initialized_(false),
      noise_ax_(9.0), noise_ay_(9.0) {
    /**
     * Initialize the EKF
     * State vector: [px, py, vx, vy]
     */
    
    // State transition matrix (will be updated at each prediction step)
    F_ = Eigen::MatrixXd::Identity(4, 4);
    
    // Measurement matrix for Lidar (only measures position)
    H_lidar_ = Eigen::MatrixXd::Zero(2, 4);
    H_lidar_(0, 0) = 1.0;  // Measure px
    H_lidar_(1, 1) = 1.0;  // Measure py
    
    // Measurement noise covariance for Lidar (standard deviations: ~0.15m)
    R_lidar_ = Eigen::MatrixXd::Zero(2, 2);
    R_lidar_(0, 0) = 0.0225;  // std_dev_lidar_x^2 = (0.15)^2
    R_lidar_(1, 1) = 0.0225;  // std_dev_lidar_y^2 = (0.15)^2
    
    // Measurement noise covariance for Radar
    R_radar_ = Eigen::MatrixXd::Zero(3, 3);
    R_radar_(0, 0) = 0.09;      // std_dev_rho^2 = (0.3)^2
    R_radar_(1, 1) = 0.0009;    // std_dev_phi^2 = (0.03)^2
    R_radar_(2, 2) = 0.09;      // std_dev_rho_dot^2 = (0.3)^2
    
    // Initialize state covariance matrix
    P_ = Eigen::MatrixXd::Identity(4, 4);
    P_(0, 0) = 1.0;   // Initial uncertainty in px
    P_(1, 1) = 1.0;   // Initial uncertainty in py
    P_(2, 2) = 1000.0; // High initial uncertainty in vx
    P_(3, 3) = 1000.0; // High initial uncertainty in vy
    
    // State vector (will be initialized with first measurement)
    x_ = Eigen::VectorXd::Zero(4);
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement) {
    if (!is_initialized_) {
        Initialize(measurement);
        previous_timestamp_ = measurement.timestamp;
        return;
    }
    
    // Calculate delta_t in seconds
    double delta_t = (measurement.timestamp - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement.timestamp;
    
    // Ensure delta_t is positive and reasonable
    if (delta_t < 0 || delta_t > 1.0) {
        std::cerr << "Warning: Unusual delta_t = " << delta_t << " seconds" << std::endl;
        delta_t = 0.0;
    }
    
    // Prediction step
    Predict(delta_t);
    
    // Update step based on sensor type
    if (measurement.sensor_type == SensorType::LIDAR) {
        UpdateLidar(measurement);
    } else if (measurement.sensor_type == SensorType::RADAR) {
        UpdateRadar(measurement);
    }
}

void FusionEKF::Initialize(const MeasurementPackage& measurement) {
    if (measurement.sensor_type == SensorType::LIDAR) {
        // Lidar provides direct Cartesian coordinates
        x_(0) = measurement.raw_measurements[0];  // px
        x_(1) = measurement.raw_measurements[1];  // py
        x_(2) = 0.0;  // vx (unknown)
        x_(3) = 0.0;  // vy (unknown)
        
        std::cout << "Filter initialized with LIDAR measurement" << std::endl;
    } else if (measurement.sensor_type == SensorType::RADAR) {
        // Radar provides polar coordinates, convert to Cartesian
        double rho = measurement.raw_measurements[0];
        double phi = measurement.raw_measurements[1];
        double rho_dot = measurement.raw_measurements[2];
        
        x_ = Tools::PolarToCartesian(rho, phi, rho_dot);
        
        std::cout << "Filter initialized with RADAR measurement" << std::endl;
    }
    
    is_initialized_ = true;
    
    std::cout << "Initial state: [" 
              << x_(0) << ", " << x_(1) << ", " 
              << x_(2) << ", " << x_(3) << "]" << std::endl;
}

void FusionEKF::Predict(double delta_t) {
    // Update state transition matrix with time step
    F_(0, 2) = delta_t;  // px += vx * dt
    F_(1, 3) = delta_t;  // py += vy * dt
    
    // Predict state
    x_ = F_ * x_;
    
    // Update process noise covariance matrix Q
    // Q accounts for uncertainty in acceleration
    double dt_squared = delta_t * delta_t;
    double dt_cubed = dt_squared * delta_t;
    double dt_quartic = dt_cubed * delta_t;
    
    Q_ = Eigen::MatrixXd::Zero(4, 4);
    Q_(0, 0) = (dt_quartic / 4.0) * noise_ax_;
    Q_(0, 2) = (dt_cubed / 2.0) * noise_ax_;
    Q_(1, 1) = (dt_quartic / 4.0) * noise_ay_;
    Q_(1, 3) = (dt_cubed / 2.0) * noise_ay_;
    Q_(2, 0) = (dt_cubed / 2.0) * noise_ax_;
    Q_(2, 2) = dt_squared * noise_ax_;
    Q_(3, 1) = (dt_cubed / 2.0) * noise_ay_;
    Q_(3, 3) = dt_squared * noise_ay_;
    
    // Predict covariance
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void FusionEKF::UpdateLidar(const MeasurementPackage& measurement) {
    // Extract measurement
    Eigen::VectorXd z(2);
    z << measurement.raw_measurements[0], measurement.raw_measurements[1];
    
    // Predict measurement
    Eigen::VectorXd z_pred = H_lidar_ * x_;
    
    // Innovation (residual)
    Eigen::VectorXd y = z - z_pred;
    
    // Innovation covariance
    Eigen::MatrixXd S = H_lidar_ * P_ * H_lidar_.transpose() + R_lidar_;
    
    // Kalman gain
    Eigen::MatrixXd K = P_ * H_lidar_.transpose() * S.inverse();
    
    // Update state
    x_ = x_ + K * y;
    
    // Update covariance using Joseph Form for numerical stability
    UpdateCovarianceJosephForm(K, H_lidar_, R_lidar_);
    
    // Compute and store NIS
    double nis = Tools::ComputeNIS(y, S);
    nis_history_lidar_.push_back(nis);
    
    // Check NIS consistency (for Lidar with 2D measurement, chi-squared critical value is 5.991)
    if (nis_history_lidar_.size() % 100 == 0) {
        double avg_nis = 0.0;
        for (double val : nis_history_lidar_) {
            avg_nis += val;
        }
        avg_nis /= nis_history_lidar_.size();
        std::cout << "Lidar NIS (avg): " << std::fixed << std::setprecision(3) 
                  << avg_nis << std::endl;
    }
}

void FusionEKF::UpdateRadar(const MeasurementPackage& measurement) {
    // Extract measurement
    Eigen::VectorXd z(3);
    z << measurement.raw_measurements[0], 
         measurement.raw_measurements[1], 
         measurement.raw_measurements[2];
    
    // Compute Jacobian matrix
    H_radar_ = Tools::CalculateJacobianRadar(x_);

    // If Jacobian collapses near origin, skip this radar update to avoid unstable gain spikes.
    if (H_radar_.isZero(1e-12)) {
        return;
    }
    
    // Predict measurement
    Eigen::VectorXd z_pred = Tools::CartesianToPolar(x_);
    
    // Innovation (residual)
    Eigen::VectorXd y = z - z_pred;
    
    // Normalize bearing angle in innovation
    y(1) = Tools::NormalizeAngle(y(1));
    
    // Innovation covariance
    Eigen::MatrixXd S = H_radar_ * P_ * H_radar_.transpose() + R_radar_;
    
    // Kalman gain
    Eigen::MatrixXd K = P_ * H_radar_.transpose() * S.inverse();
    
    // Update state
    x_ = x_ + K * y;
    
    // Update covariance using Joseph Form for numerical stability
    UpdateCovarianceJosephForm(K, H_radar_, R_radar_);
    
    // Compute and store NIS
    double nis = Tools::ComputeNIS(y, S);
    nis_history_radar_.push_back(nis);
    
    // Check NIS consistency (for Radar with 3D measurement, chi-squared critical value is 7.815)
    if (nis_history_radar_.size() % 100 == 0) {
        double avg_nis = 0.0;
        for (double val : nis_history_radar_) {
            avg_nis += val;
        }
        avg_nis /= nis_history_radar_.size();
        std::cout << "Radar NIS (avg): " << std::fixed << std::setprecision(3) 
                  << avg_nis << std::endl;
    }
}

void FusionEKF::UpdateCovarianceJosephForm(const Eigen::MatrixXd& K, 
                                          const Eigen::MatrixXd& H,
                                          const Eigen::MatrixXd& R) {
    /**
     * Joseph Form covariance update for numerical stability:
     * P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
     * 
     * This form is more numerically stable than:
     * P = (I - K*H)*P
     * 
     * as it guarantees P remains symmetric and positive-definite
     */
    int n = P_.rows();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd I_KH = I - K * H;
    
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    
    // Ensure symmetry (numerical errors can cause small asymmetries)
    P_ = (P_ + P_.transpose()) / 2.0;
}

void FusionEKF::PrintState() const {
    std::cout << "\n=== EKF State ===" << std::endl;
    std::cout << "Position (m): [" << std::fixed << std::setprecision(3)
              << x_(0) << ", " << x_(1) << "]" << std::endl;
    std::cout << "Velocity (m/s): [" << x_(2) << ", " << x_(3) << "]" << std::endl;
    std::cout << "Covariance diagonal: [" << P_(0, 0) << ", " << P_(1, 1) << ", "
              << P_(2, 2) << ", " << P_(3, 3) << "]" << std::endl;
}
