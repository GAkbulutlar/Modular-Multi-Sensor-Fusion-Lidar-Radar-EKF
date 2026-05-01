#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include "tools.h"
#include "fusion_ekf.h"

/**
 * Unit tests for EKF components
 */

// Test angle normalization
TEST(ToolsTest, NormalizeAngle) {
    // Test angle within range
    EXPECT_NEAR(Tools::NormalizeAngle(0.5), 0.5, 1e-6);
    
    // Test angle at boundaries
    EXPECT_NEAR(Tools::NormalizeAngle(M_PI), M_PI, 1e-6);
    EXPECT_NEAR(Tools::NormalizeAngle(-M_PI), -M_PI, 1e-6);
    
    // Test angle that needs wrapping (positive)
    EXPECT_NEAR(Tools::NormalizeAngle(M_PI + 0.1), -M_PI + 0.1, 1e-6);
    
    // Test angle that needs wrapping (negative)
    EXPECT_NEAR(Tools::NormalizeAngle(-M_PI - 0.1), M_PI - 0.1, 1e-6);
    
    // Test multiple rotations
    EXPECT_NEAR(Tools::NormalizeAngle(5 * M_PI), M_PI, 1e-6);
    EXPECT_NEAR(Tools::NormalizeAngle(-5 * M_PI), -M_PI, 1e-6);
}

// Test Jacobian calculation
TEST(ToolsTest, JacobianRadar) {
    // Test case 1: Object at (1, 0) with velocity (0, 1)
    Eigen::VectorXd x1(4);
    x1 << 1.0, 0.0, 0.0, 1.0;
    
    Eigen::MatrixXd J1 = Tools::CalculateJacobianRadar(x1);
    
    // Check dimensions
    EXPECT_EQ(J1.rows(), 3);
    EXPECT_EQ(J1.cols(), 4);
    
    // Jacobian should be finite
    for (int i = 0; i < J1.rows(); ++i) {
        for (int j = 0; j < J1.cols(); ++j) {
            EXPECT_TRUE(std::isfinite(J1(i, j)));
        }
    }
    
    // Test case 2: Object at (3, 4) - forms 3-4-5 triangle
    Eigen::VectorXd x2(4);
    x2 << 3.0, 4.0, 0.0, 0.0;
    
    Eigen::MatrixXd J2 = Tools::CalculateJacobianRadar(x2);
    
    // dh_rho/dx and dh_rho/dy should be px/rho and py/rho respectively
    double rho = 5.0;  // sqrt(3^2 + 4^2)
    EXPECT_NEAR(J2(0, 0), 3.0 / rho, 1e-6);  // 3/5 = 0.6
    EXPECT_NEAR(J2(0, 1), 4.0 / rho, 1e-6);  // 4/5 = 0.8
    EXPECT_NEAR(J2(0, 2), 0.0, 1e-6);
    EXPECT_NEAR(J2(0, 3), 0.0, 1e-6);
}

// Test polar to Cartesian conversion
TEST(ToolsTest, PolarToCartesian) {
    // Test case 1: 0 degrees, range 10
    Eigen::VectorXd state1 = Tools::PolarToCartesian(10.0, 0.0, 0.0);
    EXPECT_NEAR(state1(0), 10.0, 1e-6);  // px
    EXPECT_NEAR(state1(1), 0.0, 1e-6);   // py
    EXPECT_NEAR(state1(2), 0.0, 1e-6);   // vx
    EXPECT_NEAR(state1(3), 0.0, 1e-6);   // vy
    
    // Test case 2: 90 degrees, range 5
    Eigen::VectorXd state2 = Tools::PolarToCartesian(5.0, M_PI / 2, 0.0);
    EXPECT_NEAR(state2(0), 0.0, 1e-6);   // px
    EXPECT_NEAR(state2(1), 5.0, 1e-6);   // py
    EXPECT_NEAR(state2(2), 0.0, 1e-6);   // vx
    EXPECT_NEAR(state2(3), 0.0, 1e-6);   // vy
    
    // Test case 3: With radial velocity
    Eigen::VectorXd state3 = Tools::PolarToCartesian(10.0, M_PI / 4, 5.0);
    double sqrt2 = std::sqrt(2.0);
    EXPECT_NEAR(state3(0), 10.0 / sqrt2, 1e-6);      // px
    EXPECT_NEAR(state3(1), 10.0 / sqrt2, 1e-6);      // py
    EXPECT_NEAR(state3(2), 5.0 / sqrt2, 1e-6);       // vx
    EXPECT_NEAR(state3(3), 5.0 / sqrt2, 1e-6);       // vy
}

// Test Cartesian to polar conversion
TEST(ToolsTest, CartesianToPolar) {
    // Test case 1: (10, 0) with zero velocity
    Eigen::VectorXd x1(4);
    x1 << 10.0, 0.0, 0.0, 0.0;
    
    Eigen::VectorXd z1 = Tools::CartesianToPolar(x1);
    EXPECT_NEAR(z1(0), 10.0, 1e-6);      // rho
    EXPECT_NEAR(z1(1), 0.0, 1e-6);       // phi
    EXPECT_NEAR(z1(2), 0.0, 1e-6);       // rho_dot
    
    // Test case 2: (0, 5) with radial velocity
    Eigen::VectorXd x2(4);
    x2 << 0.0, 5.0, 0.0, 2.0;
    
    Eigen::VectorXd z2 = Tools::CartesianToPolar(x2);
    EXPECT_NEAR(z2(0), 5.0, 1e-6);       // rho
    EXPECT_NEAR(z2(1), M_PI / 2, 1e-6);  // phi
    EXPECT_NEAR(z2(2), 2.0, 1e-6);       // rho_dot
    
    // Test case 3: 3-4-5 triangle
    Eigen::VectorXd x3(4);
    x3 << 3.0, 4.0, 0.0, 0.0;
    
    Eigen::VectorXd z3 = Tools::CartesianToPolar(x3);
    EXPECT_NEAR(z3(0), 5.0, 1e-6);                         // rho
    EXPECT_NEAR(z3(1), std::atan2(4.0, 3.0), 1e-6);        // phi
    EXPECT_NEAR(z3(2), 0.0, 1e-6);                         // rho_dot
}

// Test NIS computation
TEST(ToolsTest, ComputeNIS) {
    // Create a simple innovation and covariance
    Eigen::VectorXd innovation(2);
    innovation << 1.0, 0.0;
    
    Eigen::MatrixXd S(2, 2);
    S << 1.0, 0.0,
         0.0, 1.0;
    
    double nis = Tools::ComputeNIS(innovation, S);
    // For innovation [1, 0] and S = I, NIS = 1^2 + 0^2 = 1
    EXPECT_NEAR(nis, 1.0, 1e-6);
    
    // Test with correlation
    Eigen::VectorXd innovation2(2);
    innovation2 << 2.0, 0.0;
    
    double nis2 = Tools::ComputeNIS(innovation2, S);
    // NIS = 2^2 + 0^2 = 4
    EXPECT_NEAR(nis2, 4.0, 1e-6);
}

// Test EKF initialization
TEST(FusionEKFTest, InitializationLidar) {
    FusionEKF ekf;
    
    MeasurementPackage lidar_meas(SensorType::LIDAR, 1000000);
    lidar_meas.raw_measurements[0] = 10.0;
    lidar_meas.raw_measurements[1] = 20.0;
    
    EXPECT_FALSE(ekf.IsInitialized());
    
    ekf.ProcessMeasurement(lidar_meas);
    
    EXPECT_TRUE(ekf.IsInitialized());
    auto state = ekf.GetState();
    EXPECT_NEAR(state(0), 10.0, 1e-6);  // px
    EXPECT_NEAR(state(1), 20.0, 1e-6);  // py
}

TEST(FusionEKFTest, InitializationRadar) {
    FusionEKF ekf;
    
    MeasurementPackage radar_meas(SensorType::RADAR, 1000000);
    radar_meas.raw_measurements[0] = 10.0;  // rho
    radar_meas.raw_measurements[1] = M_PI / 4;  // phi (45 degrees)
    radar_meas.raw_measurements[2] = 0.0;   // rho_dot
    
    EXPECT_FALSE(ekf.IsInitialized());
    
    ekf.ProcessMeasurement(radar_meas);
    
    EXPECT_TRUE(ekf.IsInitialized());
    auto state = ekf.GetState();
    // Cartesian: (10*cos(45), 10*sin(45)) = (7.07, 7.07)
    double expected = 10.0 / std::sqrt(2.0);
    EXPECT_NEAR(state(0), expected, 1e-3);  // px
    EXPECT_NEAR(state(1), expected, 1e-3);  // py
}

// Test EKF prediction
TEST(FusionEKFTest, Prediction) {
    FusionEKF ekf;
    
    // Initialize with Lidar
    MeasurementPackage init_meas(SensorType::LIDAR, 0);
    init_meas.raw_measurements[0] = 0.0;
    init_meas.raw_measurements[1] = 0.0;
    ekf.ProcessMeasurement(init_meas);
    
    // Set initial velocity
    Eigen::VectorXd state = ekf.GetState();
    state(2) = 1.0;  // vx = 1 m/s
    state(3) = 0.0;  // vy = 0
    
    // Process another measurement after 1 second
    MeasurementPackage meas2(SensorType::LIDAR, 1000000);  // 1 second later
    meas2.raw_measurements[0] = 1.0;  // Should be roughly at x=1
    meas2.raw_measurements[1] = 0.0;
    
    ekf.ProcessMeasurement(meas2);
    
    auto updated_state = ekf.GetState();
    // After prediction, should have moved roughly 1 meter in x direction
    EXPECT_GT(updated_state(0), 0.5);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
