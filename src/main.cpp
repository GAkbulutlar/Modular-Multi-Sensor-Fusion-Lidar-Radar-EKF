#include "fusion_ekf.h"
#include "measurement.h"
#include "tools.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <random>
#include <iomanip>
#include <vector>
#include <limits>

/**
 * @struct SimulationResult
 * @brief Stores fusion results for analysis
 */
struct SimulationResult {
    long long timestamp;
    double gt_x, gt_y, gt_vx, gt_vy;        // Ground truth
    double fused_x, fused_y, fused_vx, fused_vy;  // Fused estimate
    double lidar_x, lidar_y;                      // Lidar-only estimate
    double radar_x, radar_y;                      // Radar-only estimate
};

/**
 * @class TargetTrajectory
 * @brief Generates circular motion trajectory
 */
class TargetTrajectory {
private:
    double center_x_;
    double center_y_;
    double radius_;
    double angular_velocity_;  // rad/s

public:
    TargetTrajectory(double cx, double cy, double r, double omega)
        : center_x_(cx), center_y_(cy), radius_(r),
          angular_velocity_(omega) {}

    /**
     * Get ground truth state at time t
     * @param t Time in seconds
     * @return State vector [px, py, vx, vy]
     */
    Eigen::VectorXd GetState(double t) const {
        double angle = angular_velocity_ * t;

        double px = center_x_ + radius_ * std::cos(angle);
        double py = center_y_ + radius_ * std::sin(angle);

        // Velocity is tangent to circle
        double vx = -radius_ * angular_velocity_ * std::sin(angle);
        double vy = radius_ * angular_velocity_ * std::cos(angle);

        Eigen::VectorXd state(4);
        state << px, py, vx, vy;
        return state;
    }
};

/**
 * @class MeasurementGenerator
 * @brief Generates noisy sensor measurements
 */
class MeasurementGenerator {
private:
    std::mt19937 rng_;
    std::normal_distribution<double> lidar_dist_x_;
    std::normal_distribution<double> lidar_dist_y_;
    std::normal_distribution<double> radar_dist_rho_;
    std::normal_distribution<double> radar_dist_phi_;
    std::normal_distribution<double> radar_dist_rho_dot_;

public:
    MeasurementGenerator(unsigned int seed = 42)
        : rng_(seed),
          lidar_dist_x_(0.0, 0.15),   // std_dev = 0.15m
          lidar_dist_y_(0.0, 0.15),
          radar_dist_rho_(0.0, 0.3),  // std_dev = 0.3m
          radar_dist_phi_(0.0, 0.03), // std_dev = 0.03 rad (~1.7 degrees)
          radar_dist_rho_dot_(0.0, 0.3) {} // std_dev = 0.3 m/s

    /**
     * Generate Lidar measurement with noise
     * @param gt Ground truth state
     * @return MeasurementPackage with Lidar data
     */
    MeasurementPackage GenerateLidarMeasurement(long long timestamp,
                                               const Eigen::VectorXd& gt) {
        MeasurementPackage pkg(SensorType::LIDAR, timestamp);
        pkg.raw_measurements[0] = gt(0) + lidar_dist_x_(rng_);
        pkg.raw_measurements[1] = gt(1) + lidar_dist_y_(rng_);
        return pkg;
    }

    /**
     * Generate Radar measurement with noise
     * @param gt Ground truth state
     * @return MeasurementPackage with Radar data
     */
    MeasurementPackage GenerateRadarMeasurement(long long timestamp,
                                               const Eigen::VectorXd& gt) {
        MeasurementPackage pkg(SensorType::RADAR, timestamp);

        // Convert ground truth to polar
        double px = gt(0);
        double py = gt(1);
        double vx = gt(2);
        double vy = gt(3);

        double rho = std::sqrt(px * px + py * py);
        double phi = std::atan2(py, px);
        double rho_dot = (rho > 1e-9) ? (px * vx + py * vy) / rho : 0.0;

        // Add noise
        pkg.raw_measurements[0] = rho + radar_dist_rho_(rng_);
        pkg.raw_measurements[1] = phi + radar_dist_phi_(rng_);
        pkg.raw_measurements[2] = rho_dot + radar_dist_rho_dot_(rng_);

        return pkg;
    }
};

int main() {
    std::cout << "=== Multi-Sensor Fusion EKF Simulation ===" << std::endl;
    std::cout << "Target Motion: Circular path" << std::endl;
    std::cout << "Lidar Frequency: 10 Hz" << std::endl;
    std::cout << "Radar Frequency: 20 Hz (offset 0.025s)" << std::endl;
    std::cout << std::endl;

    // Simulation parameters
    const double simulation_time = 20.0;  // seconds
    const double lidar_period = 0.1;     // 10 Hz
    const double radar_period = 0.05;    // 20 Hz
    const double radar_offset = 0.025;   // 25ms offset
    const double dt = 0.01;              // Global time step (10ms)

    // Trajectory parameters: circular motion
    const double center_x = 0.0;
    const double center_y = 0.0;
    const double radius = 50.0;          // meters
    const double speed = 10.0;           // m/s
    const double angular_velocity = speed / radius;  // rad/s

    // Create trajectory and measurement generator
    TargetTrajectory trajectory(center_x, center_y, radius, angular_velocity);
    MeasurementGenerator meas_gen(42);  // Fixed seed for reproducibility

    // Create filters
    FusionEKF ekf_fused;
    FusionEKF ekf_lidar_only;
    FusionEKF ekf_radar_only;

    // Storage for results
    std::vector<SimulationResult> results;

    // Run simulation
    double lidar_next_time = 0.0;
    double radar_next_time = radar_offset;

    for (double t = 0.0; t <= simulation_time; t += dt) {
        long long timestamp_us = static_cast<long long>(t * 1e6);

        // Get ground truth
        Eigen::VectorXd gt = trajectory.GetState(t);

        SimulationResult result{};
        result.timestamp = timestamp_us;
        result.gt_x = gt(0);
        result.gt_y = gt(1);
        result.gt_vx = gt(2);
        result.gt_vy = gt(3);
        result.fused_x = std::numeric_limits<double>::quiet_NaN();
        result.fused_y = std::numeric_limits<double>::quiet_NaN();
        result.fused_vx = std::numeric_limits<double>::quiet_NaN();
        result.fused_vy = std::numeric_limits<double>::quiet_NaN();
        result.lidar_x = std::numeric_limits<double>::quiet_NaN();
        result.lidar_y = std::numeric_limits<double>::quiet_NaN();
        result.radar_x = std::numeric_limits<double>::quiet_NaN();
        result.radar_y = std::numeric_limits<double>::quiet_NaN();

        // Generate and process Lidar measurement
        if (t >= lidar_next_time) {
            MeasurementPackage lidar_pkg = meas_gen.GenerateLidarMeasurement(timestamp_us, gt);

            ekf_fused.ProcessMeasurement(lidar_pkg);
            ekf_lidar_only.ProcessMeasurement(lidar_pkg);

            lidar_next_time += lidar_period;
        }

        // Generate and process Radar measurement
        if (t >= radar_next_time) {
            MeasurementPackage radar_pkg = meas_gen.GenerateRadarMeasurement(timestamp_us, gt);

            ekf_fused.ProcessMeasurement(radar_pkg);
            ekf_radar_only.ProcessMeasurement(radar_pkg);

            radar_next_time += radar_period;
        }

        // Store results if filter is initialized
        if (ekf_fused.IsInitialized()) {
            auto state = ekf_fused.GetState();
            result.fused_x = state(0);
            result.fused_y = state(1);
            result.fused_vx = state(2);
            result.fused_vy = state(3);

            if (ekf_lidar_only.IsInitialized()) {
                auto state_lidar = ekf_lidar_only.GetState();
                result.lidar_x = state_lidar(0);
                result.lidar_y = state_lidar(1);
            }

            if (ekf_radar_only.IsInitialized()) {
                auto state_radar = ekf_radar_only.GetState();
                result.radar_x = state_radar(0);
                result.radar_y = state_radar(1);
            }

            results.push_back(result);
        }
    }

    std::cout << "Simulation complete. Processed " << results.size() << " samples." << std::endl;
    std::cout << std::endl;

    // Calculate and display errors
    std::cout << "=== Error Analysis ===" << std::endl;

    double fused_rmse_pos = 0.0;
    double lidar_rmse_pos = 0.0;
    double radar_rmse_pos = 0.0;
    std::size_t fused_count = 0;
    std::size_t lidar_count = 0;
    std::size_t radar_count = 0;

    for (const auto& result : results) {
        if (std::isfinite(result.fused_x) && std::isfinite(result.fused_y)) {
            double fused_err = std::sqrt(
                std::pow(result.fused_x - result.gt_x, 2) +
                std::pow(result.fused_y - result.gt_y, 2)
            );
            fused_rmse_pos += fused_err * fused_err;
            ++fused_count;
        }

        if (std::isfinite(result.lidar_x) && std::isfinite(result.lidar_y)) {
            double lidar_err = std::sqrt(
                std::pow(result.lidar_x - result.gt_x, 2) +
                std::pow(result.lidar_y - result.gt_y, 2)
            );
            lidar_rmse_pos += lidar_err * lidar_err;
            ++lidar_count;
        }

        if (std::isfinite(result.radar_x) && std::isfinite(result.radar_y)) {
            double radar_err = std::sqrt(
                std::pow(result.radar_x - result.gt_x, 2) +
                std::pow(result.radar_y - result.gt_y, 2)
            );
            radar_rmse_pos += radar_err * radar_err;
            ++radar_count;
        }
    }

    fused_rmse_pos = (fused_count > 0) ? std::sqrt(fused_rmse_pos / fused_count) : 0.0;
    lidar_rmse_pos = (lidar_count > 0) ? std::sqrt(lidar_rmse_pos / lidar_count) : 0.0;
    radar_rmse_pos = (radar_count > 0) ? std::sqrt(radar_rmse_pos / radar_count) : 0.0;

    std::cout << "Position RMSE:" << std::endl;
    std::cout << "  Fused:       " << std::fixed << std::setprecision(4)
              << fused_rmse_pos << " m" << std::endl;
    std::cout << "  Lidar-only:  " << lidar_rmse_pos << " m" << std::endl;
    std::cout << "  Radar-only:  " << radar_rmse_pos << " m" << std::endl;
    std::cout << std::endl;

    // Print NIS statistics
    auto nis_lidar = ekf_fused.GetNISHistoryLidar();
    auto nis_radar = ekf_fused.GetNISHistoryRadar();

    std::cout << "=== NIS Consistency Check ===" << std::endl;
    if (!nis_lidar.empty()) {
        double lidar_nis_avg = 0.0;
        for (double nis : nis_lidar) {
            lidar_nis_avg += nis;
        }
        lidar_nis_avg /= nis_lidar.size();
        std::cout << "Lidar NIS count:  " << nis_lidar.size() << std::endl;
        std::cout << "Lidar NIS avg:    " << std::fixed << std::setprecision(4)
                  << lidar_nis_avg << std::endl;
        std::cout << "  (Expected: ~2.0, 95% should be < 5.991)" << std::endl;
    }

    if (!nis_radar.empty()) {
        double radar_nis_avg = 0.0;
        for (double nis : nis_radar) {
            radar_nis_avg += nis;
        }
        radar_nis_avg /= nis_radar.size();
        std::cout << "Radar NIS count:  " << nis_radar.size() << std::endl;
        std::cout << "Radar NIS avg:    " << std::fixed << std::setprecision(4)
                  << radar_nis_avg << std::endl;
        std::cout << "  (Expected: ~3.0, 95% should be < 7.815)" << std::endl;
    }
    std::cout << std::endl;

    // Write results to CSV for Python analysis
    std::ofstream csv_file("fusion_results.csv");
    csv_file << "time,gt_x,gt_y,gt_vx,gt_vy,"
             << "fused_x,fused_y,fused_vx,fused_vy,"
             << "lidar_x,lidar_y,radar_x,radar_y\n";

    for (const auto& result : results) {
        csv_file << std::fixed << std::setprecision(6)
                 << result.timestamp / 1e6 << ","
                 << result.gt_x << "," << result.gt_y << ","
                 << result.gt_vx << "," << result.gt_vy << ","
                 << result.fused_x << "," << result.fused_y << ","
                 << result.fused_vx << "," << result.fused_vy << ","
                 << result.lidar_x << "," << result.lidar_y << ","
                 << result.radar_x << "," << result.radar_y << "\n";
    }
    csv_file.close();

    std::cout << "Results saved to fusion_results.csv" << std::endl;

    return 0;
}
