#ifndef CONFIG_H
#define CONFIG_H

/**
 * @file config.h
 * @brief Configuration parameters for the EKF system
 */

#include <cmath>

namespace Config {

// =============================================================================
// SIMULATION PARAMETERS
// =============================================================================

constexpr double SIMULATION_TIME = 20.0;  // seconds
constexpr double SIMULATION_DT = 0.01;    // 10ms global time step

// Trajectory parameters
constexpr double TRAJECTORY_CENTER_X = 0.0;    // meters
constexpr double TRAJECTORY_CENTER_Y = 0.0;    // meters
constexpr double TRAJECTORY_RADIUS = 50.0;    // meters
constexpr double TRAJECTORY_SPEED = 10.0;     // m/s

// =============================================================================
// SENSOR PARAMETERS
// =============================================================================

// Lidar configuration
constexpr double LIDAR_FREQUENCY = 10.0;      // Hz
constexpr double LIDAR_PERIOD = 1.0 / LIDAR_FREQUENCY;  // seconds
constexpr double LIDAR_STD_DEV_X = 0.15;      // meters
constexpr double LIDAR_STD_DEV_Y = 0.15;      // meters
constexpr double LIDAR_NOISE_VAR_X = LIDAR_STD_DEV_X * LIDAR_STD_DEV_X;
constexpr double LIDAR_NOISE_VAR_Y = LIDAR_STD_DEV_Y * LIDAR_STD_DEV_Y;

// Radar configuration
constexpr double RADAR_FREQUENCY = 20.0;      // Hz
constexpr double RADAR_PERIOD = 1.0 / RADAR_FREQUENCY;  // seconds
constexpr double RADAR_OFFSET = 0.025;        // seconds (25ms offset from Lidar)
constexpr double RADAR_STD_DEV_RHO = 0.3;     // meters
constexpr double RADAR_STD_DEV_PHI = 0.03;    // radians (~1.7 degrees)
constexpr double RADAR_STD_DEV_RHO_DOT = 0.3; // m/s
constexpr double RADAR_NOISE_VAR_RHO = RADAR_STD_DEV_RHO * RADAR_STD_DEV_RHO;
constexpr double RADAR_NOISE_VAR_PHI = RADAR_STD_DEV_PHI * RADAR_STD_DEV_PHI;
constexpr double RADAR_NOISE_VAR_RHO_DOT = RADAR_STD_DEV_RHO_DOT * RADAR_STD_DEV_RHO_DOT;

// =============================================================================
// EKF PARAMETERS
// =============================================================================

// Process noise (uncertainty in acceleration)
constexpr double PROCESS_NOISE_AX = 9.0;      // m/s^2
constexpr double PROCESS_NOISE_AY = 9.0;      // m/s^2

// Initial state covariance
constexpr double INITIAL_COV_PX = 1.0;        // position x
constexpr double INITIAL_COV_PY = 1.0;        // position y
constexpr double INITIAL_COV_VX = 1000.0;     // velocity x (high uncertainty)
constexpr double INITIAL_COV_VY = 1000.0;     // velocity y (high uncertainty)

// =============================================================================
// NUMERICAL PARAMETERS
// =============================================================================

constexpr double ANGLE_EPSILON = 1e-9;        // Small angle threshold
constexpr double DISTANCE_EPSILON = 1e-6;     // Small distance threshold
constexpr double MATRIX_SYMMETRY_EPSILON = 1e-10;  // For covariance matrix symmetry

// =============================================================================
// OUTPUT PARAMETERS
// =============================================================================

constexpr const char* CSV_OUTPUT_FILE = "fusion_results.csv";
constexpr const char* METRICS_OUTPUT_FILE = "metrics.txt";

// NIS statistics for consistency checking
// Chi-squared critical values (95% confidence):
// - df=2 (Lidar): 5.991
// - df=3 (Radar): 7.815
constexpr double NIS_LIDAR_THRESHOLD = 5.991;
constexpr double NIS_RADAR_THRESHOLD = 7.815;

// =============================================================================
// LOGGING PARAMETERS
// =============================================================================

constexpr int NIS_LOG_INTERVAL = 100;  // Log NIS every N measurements

} // namespace Config

#endif // CONFIG_H
