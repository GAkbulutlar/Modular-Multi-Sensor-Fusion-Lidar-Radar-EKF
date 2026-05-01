# System Architecture and Design

## Overview

The Multi-Sensor Fusion EKF is a modular, production-ready system for fusing Lidar and Radar measurements to estimate 2D position and velocity of a moving target.

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Measurement Stream                        │
│          (Lidar @ 10Hz, Radar @ 20Hz, asynchronous)         │
└──────────────┬──────────────────────────────────────┬────────┘
               │                                      │
               ▼                                      ▼
        ┌─────────────┐                      ┌─────────────┐
        │ Lidar Data  │                      │ Radar Data  │
        │  (x, y)     │                      │ (ρ,φ,ρ̇)    │
        └─────────────┘                      └─────────────┘
               │                                      │
               └──────────────────┬───────────────────┘
                                  │
                    ┌─────────────▼──────────────┐
                    │   MeasurementPackage       │
                    │  - sensor_type             │
                    │  - timestamp               │
                    │  - raw_measurements[]      │
                    └─────────────┬──────────────┘
                                  │
                    ┌─────────────▼──────────────────┐
                    │     FusionEKF::               │
                    │  ProcessMeasurement()          │
                    └─────────────┬──────────────────┘
                                  │
                    ┌─────────────▼──────────────────────┐
                    │        EKF Pipeline                │
                    │  1. Prediction (F, Q matrices)      │
                    │  2. Update (H, R, K computation)    │
                    │  3. Joseph Form covariance update   │
                    └─────────────┬──────────────────────┘
                                  │
                    ┌─────────────▼──────────────────┐
                    │   State Estimate               │
                    │  [px, py, vx, vy]ᵀ             │
                    │  Covariance Matrix P           │
                    └─────────────┬──────────────────┘
                                  │
                    ┌─────────────▼──────────────────┐
                    │   Output & Analysis            │
                    │  - State vectors               │
                    │  - NIS values                  │
                    │  - CSV file export             │
                    └────────────────────────────────┘
```

## Class Hierarchy

### FusionEKF (Core Filter)
```
┌─────────────────────────────────────────┐
│           FusionEKF                     │
├─────────────────────────────────────────┤
│ Private Members:                        │
│  x_: VectorXd          [px,py,vx,vy]   │
│  P_: MatrixXd          4x4 covariance  │
│  F_: MatrixXd          4x4 state model │
│  Q_: MatrixXd          4x4 process noise
│  H_lidar_: MatrixXd    2x4 measurement │
│  H_radar_: MatrixXd    3x4 Jacobian    │
│  R_lidar_, R_radar_    noise matrices  │
│  nis_history_*         validation data │
├─────────────────────────────────────────┤
│ Public Methods:                         │
│  ProcessMeasurement()   main entry point│
│  GetState()             current estimate│
│  GetCovariance()        uncertainty     │
│  IsInitialized()        status check    │
│  PrintState()           debugging       │
├─────────────────────────────────────────┤
│ Private Methods:                        │
│  Initialize()           first measurement
│  Predict()              time update     │
│  UpdateLidar()          measurement upd │
│  UpdateRadar()          measurement upd │
│  UpdateCovarianceJoseph() stability    │
└─────────────────────────────────────────┘
```

### MeasurementPackage
```
┌──────────────────────────────┐
│  MeasurementPackage          │
├──────────────────────────────┤
│  sensor_type: SensorType     │
│    - LIDAR                   │
│    - RADAR                   │
│  timestamp: long long (μs)   │
│  raw_measurements[3]         │
│    - Lidar: [x, y, *]        │
│    - Radar: [ρ, φ, ρ̇]      │
└──────────────────────────────┘
```

### Tools (Utility Functions)
```
┌─────────────────────────────────────────┐
│           Tools (Static)                 │
├─────────────────────────────────────────┤
│  NormalizeAngle()                        │
│    - Wraps angle to [-π, π]              │
│  CalculateJacobianRadar()                │
│    - Computes H matrix for Radar         │
│  ComputeNIS()                            │
│    - Normalized Innovation Squared       │
│  PolarToCartesian()                      │
│    - Coordinate transform                │
│  CartesianToPolar()                      │
│    - Inverse coordinate transform        │
└─────────────────────────────────────────┘
```

## Data Flow

### Initialization Phase
```
First Measurement Arrives
       ↓
Determine Sensor Type
       ↓
   ┌─────────────────────────────────────┐
   │                                       │
   ▼ (LIDAR)                ▼ (RADAR)     │
Set x = [px, py, 0, 0]   x = PolarToCart │
(px, py from measurement) (rho, phi, rho_dot)
   │                           │           │
   └─────────────┬─────────────┘           │
                 ▼                         │
        Set P = I·diag([1, 1, 1000, 1000])│
                 ▼                         │
        is_initialized_ = true             │
```

### Processing Phase (Per Measurement)
```
MeasurementPackage Received
       ↓
Check Initialization Status
       ├─ If NOT initialized: Initialize()
       │
       └─ If initialized:
          ├─ Calculate Δt = timestamp_now - timestamp_prev
          │
          ├─ PREDICT STEP
          │  ├─ Update F_ with Δt
          │  ├─ Compute Q_ from noise parameters
          │  ├─ x_ = F_ × x_
          │  └─ P_ = F_ × P_ × F_ᵀ + Q_
          │
          ├─ UPDATE STEP (depends on sensor)
          │  │
          │  ├─ LIDAR PATH:
          │  │  ├─ z = measurement
          │  │  ├─ z_pred = H_lidar × x_
          │  │  ├─ y = z - z_pred (innovation)
          │  │  ├─ S = H_lidar × P × H_lidarᵀ + R_lidar
          │  │  ├─ K = P × H_lidarᵀ × S⁻¹ (Kalman gain)
          │  │  ├─ x_ = x_ + K × y
          │  │  ├─ Joseph Form Update
          │  │  └─ Compute NIS
          │  │
          │  └─ RADAR PATH:
          │     ├─ z = [rho, phi, rho_dot]ᵀ
          │     ├─ H_radar_ = Jacobian(x_)
          │     ├─ z_pred = CartesianToPolar(x_)
          │     ├─ y = z - z_pred
          │     ├─ Normalize y[1] (bearing angle)
          │     ├─ S = H_radar × P × H_radarᵀ + R_radar
          │     ├─ K = P × H_radarᵀ × S⁻¹
          │     ├─ x_ = x_ + K × y
          │     ├─ Joseph Form Update
          │     └─ Compute NIS
```

## Mathematical Model

### State Space Representation

**State Vector:**
$$\mathbf{x} = \begin{bmatrix} p_x \\ p_y \\ v_x \\ v_y \end{bmatrix}$$

**Discrete Process Model (Constant Velocity):**
$$\mathbf{x}_{k+1} = \mathbf{F}_k \mathbf{x}_k + \mathbf{w}_k$$

where:
$$\mathbf{F}_k = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}, \quad \mathbf{w}_k \sim \mathcal{N}(0, \mathbf{Q}_k)$$

**Process Noise Covariance:**
$$\mathbf{Q}_k = \begin{bmatrix} \frac{\Delta t^4}{4}a_x & 0 & \frac{\Delta t^3}{2}a_x & 0 \\ 0 & \frac{\Delta t^4}{4}a_y & 0 & \frac{\Delta t^3}{2}a_y \\ \frac{\Delta t^3}{2}a_x & 0 & \Delta t^2 a_x & 0 \\ 0 & \frac{\Delta t^3}{2}a_y & 0 & \Delta t^2 a_y \end{bmatrix}$$

### Measurement Models

**Lidar (Cartesian):**
$$\mathbf{z}_{lidar} = \mathbf{H}_{lidar} \mathbf{x} + \mathbf{v}_{lidar}$$

where:
$$\mathbf{H}_{lidar} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

**Radar (Polar):**
$$\mathbf{z}_{radar} = h(\mathbf{x}) + \mathbf{v}_{radar}$$

where:
$$h(\mathbf{x}) = \begin{bmatrix} \sqrt{p_x^2 + p_y^2} \\ \text{atan2}(p_y, p_x) \\ \frac{p_x v_x + p_y v_y}{\sqrt{p_x^2 + p_y^2}} \end{bmatrix}$$

$$\mathbf{H}_{radar} = \frac{\partial h}{\partial \mathbf{x}}\bigg|_{\mathbf{x}}$$

### Extended Kalman Filter Equations

**Prediction:**
- $$\hat{\mathbf{x}}^{-}_k = \mathbf{F}_{k-1} \hat{\mathbf{x}}_{k-1}$$
- $$\mathbf{P}^{-}_k = \mathbf{F}_{k-1} \mathbf{P}_{k-1} \mathbf{F}^T_{k-1} + \mathbf{Q}_{k-1}$$

**Update:**
- $$\mathbf{y}_k = \mathbf{z}_k - h(\hat{\mathbf{x}}^{-}_k)$$
- $$\mathbf{S}_k = \mathbf{H}_k \mathbf{P}^{-}_k \mathbf{H}^T_k + \mathbf{R}_k$$
- $$\mathbf{K}_k = \mathbf{P}^{-}_k \mathbf{H}^T_k \mathbf{S}^{-1}_k$$
- $$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}^{-}_k + \mathbf{K}_k \mathbf{y}_k$$
- $$\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}^{-}_k (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}^T_k$$ (Joseph Form)

## Design Decisions

### 1. Joseph Form Covariance Update
**Why**: Standard update $(I - KH)P$ can become non-symmetric or non-positive-definite due to numerical errors.

**Solution**: Joseph Form guarantees symmetry and positive-definiteness.

### 2. Angle Normalization
**Why**: Bearing angles in Radar measurements wrap at ±π, causing discontinuities.

**Solution**: Normalize all bearing angles to [-π, π] range before and after computations.

### 3. Jacobian Computation for Radar
**Why**: Radar measurement is nonlinear; requires Jacobian for EKF.

**Solution**: Compute Jacobian analytically at each update step, with division-by-zero protection.

### 4. Asynchronous Sensor Processing
**Why**: Real sensors have different frequencies and timing.

**Solution**: Track previous timestamp, compute Δt, update state matrices per measurement.

### 5. Modular Architecture
**Why**: Easier to test, extend, and maintain.

**Solution**: Separate concerns into FusionEKF (core), Tools (utilities), MeasurementPackage (data).

## Testing Strategy

### Unit Tests
- **Angle Normalization**: Boundary cases (±π, multiples of 2π)
- **Jacobian**: Numerical verification against finite differences
- **Coordinate Transforms**: Round-trip conversion accuracy
- **NIS Computation**: Known values validation
- **Filter Initialization**: Both Lidar and Radar paths

### Integration Tests (Simulation)
- **Circular Motion**: Predictable trajectory for validation
- **Multi-Sensor Fusion**: Compare against single-sensor-only filters
- **NIS Consistency**: Verify ~95% of values within expected ranges
- **Error Metrics**: Quantify fusion benefits

### Performance Metrics
- **Position RMSE**: Root Mean Squared Error in meters
- **Velocity RMSE**: Velocity estimation accuracy
- **Convergence Time**: How quickly filter converges after initialization
- **Filter Health**: NIS average and distribution

## Extension Points

### Adding a Third Sensor (e.g., GPS)
1. Add to `SensorType` enum in `measurement.h`
2. Create measurement matrix in `fusion_ekf.cpp`
3. Add update method `UpdateGPS()`
4. Implement coordinate transformation in `tools.cpp`

### Adaptive Process Noise
```cpp
// Could use estimated innovation magnitude
double innovation_norm = y.norm();
noise_ax_ = base_noise_ax * (1.0 + alpha * innovation_norm);
```

### Kalman Smoother
```cpp
// Backward pass for improved state estimates
std::vector<VectorXd> x_forward, x_smooth;
// ... collect forward estimates
// Then run backward pass
```

### Multi-Target Tracking
```cpp
// Data Association
for (const auto& measurement : measurements) {
    target = AssociateToTarget(measurement);
    ekf[target].ProcessMeasurement(measurement);
}
```

## Performance Characteristics

- **Memory**: ~200 bytes per filter instance (4x4 matrices)
- **CPU**: ~1ms per measurement on modern CPU
- **Latency**: <1ms update to output
- **Convergence**: ~10 measurements to reasonable accuracy

## Numerical Stability Considerations

1. **Matrix Conditioning**: Covariance matrices well-conditioned
2. **Division by Zero**: Protected in Jacobian and coordinate transforms
3. **Angle Wrapping**: Prevents discontinuities in bearing
4. **Symmetry Enforcement**: Joseph form maintains covariance properties
5. **Singular Matrix Protection**: Check determinants before inversion

## References

See README.md for full reference list.
