#ifndef FUSION_EKF_H
#define FUSION_EKF_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "measurement.h"

/**
 * @class FusionEKF
 * @brief Extended Kalman Filter for fusing Lidar and Radar measurements
 * 
 * This class implements a 2D EKF for tracking a moving target using
 * both Lidar (Cartesian) and Radar (polar) measurements.
 * 
 * State vector: [px, py, vx, vy]
 * - px, py: position in meters
 * - vx, vy: velocity in meters per second
 */
class FusionEKF {
private:
    // State vector and covariance matrix
    Eigen::VectorXd x_;           ///< State vector [px, py, vx, vy]
    Eigen::MatrixXd P_;           ///< State covariance matrix (4x4)
    
    // Measurement matrices
    Eigen::MatrixXd H_lidar_;     ///< Lidar measurement matrix (2x4)
    Eigen::MatrixXd H_radar_;     ///< Radar Jacobian (computed for each update)
    
    // Process and measurement noise
    Eigen::MatrixXd Q_;           ///< Process noise covariance (4x4)
    Eigen::MatrixXd R_lidar_;     ///< Lidar measurement noise (2x2)
    Eigen::MatrixXd R_radar_;     ///< Radar measurement noise (3x3)
    
    // Transition matrix
    Eigen::MatrixXd F_;           ///< State transition matrix (4x4)
    
    // Process noise parameters
    double noise_ax_;             ///< Acceleration noise in x (m/s^2)
    double noise_ay_;             ///< Acceleration noise in y (m/s^2)
    
    // Timestamps
    long long previous_timestamp_; ///< Previous measurement timestamp in microseconds
    bool is_initialized_;         ///< Flag indicating if filter is initialized
    
    // NIS history for validation
    std::vector<double> nis_history_lidar_;   ///< NIS values for Lidar
    std::vector<double> nis_history_radar_;   ///< NIS values for Radar
    
public:
    /**
     * @brief Constructor for FusionEKF
     */
    FusionEKF();
    
    /**
     * @brief Destructor
     */
    ~FusionEKF() = default;
    
    /**
     * @brief Process a measurement package and update the state
     * @param measurement MeasurementPackage containing sensor data and timestamp
     */
    void ProcessMeasurement(const MeasurementPackage& measurement);
    
    /**
     * @brief Get the current state vector
     * @return Reference to state vector [px, py, vx, vy]
     */
    const Eigen::VectorXd& GetState() const { return x_; }
    
    /**
     * @brief Get the current covariance matrix
     * @return Reference to covariance matrix P
     */
    const Eigen::MatrixXd& GetCovariance() const { return P_; }
    
    /**
     * @brief Check if filter is initialized
     * @return True if initialized, false otherwise
     */
    bool IsInitialized() const { return is_initialized_; }
    
    /**
     * @brief Get NIS history for Lidar measurements
     * @return Vector of NIS values
     */
    const std::vector<double>& GetNISHistoryLidar() const { 
        return nis_history_lidar_; 
    }
    
    /**
     * @brief Get NIS history for Radar measurements
     * @return Vector of NIS values
     */
    const std::vector<double>& GetNISHistoryRadar() const { 
        return nis_history_radar_; 
    }
    
    /**
     * @brief Print current state and covariance for debugging
     */
    void PrintState() const;

private:
    /**
     * @brief Initialize state vector from first measurement
     * @param measurement First measurement package
     */
    void Initialize(const MeasurementPackage& measurement);
    
    /**
     * @brief Predict state and covariance using process model
     * @param delta_t Time elapsed since last measurement (seconds)
     */
    void Predict(double delta_t);
    
    /**
     * @brief Update state with Lidar measurement
     * @param measurement Lidar measurement package
     */
    void UpdateLidar(const MeasurementPackage& measurement);
    
    /**
     * @brief Update state with Radar measurement
     * @param measurement Radar measurement package
     */
    void UpdateRadar(const MeasurementPackage& measurement);
    
    /**
     * @brief Compute and apply Joseph Form covariance update for numerical stability
     * @param K Kalman gain matrix
     * @param H Measurement matrix (Jacobian for Radar)
     * @param R Measurement noise covariance
     */
    void UpdateCovarianceJosephForm(const Eigen::MatrixXd& K, 
                                   const Eigen::MatrixXd& H,
                                   const Eigen::MatrixXd& R);
};

#endif // FUSION_EKF_H
