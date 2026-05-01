# Quick Start Guide

## Windows Build (MSVC)

### 1. Install Dependencies

#### Option A: Using vcpkg (Recommended)
```bash
# Clone vcpkg
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat

# Install packages
.\vcpkg install eigen3:x64-windows gtest:x64-windows

# Note the integration path shown
```

#### Option B: Manual Installation
- Download Eigen3 from https://eigen.tuxfamily.org
- Download GoogleTest from https://github.com/google/googletest
- Extract to appropriate system directories

### 2. Configure and Build

```bash
cd "Modular Multi-Sensor Fusion (Lidar + Radar) EKF"
mkdir build
cd build

# If using vcpkg
cmake .. -G "Visual Studio 16 2019" -A x64 ^
  -DCMAKE_TOOLCHAIN_FILE=<path-to-vcpkg>/scripts/buildsystems/vcpkg.cmake

# Or if packages are in system path
cmake .. -G "Visual Studio 16 2019" -A x64

cmake --build . --config Release -j8
```

### 3. Run Simulation and Tests

```bash
# Run the main simulation
.\bin\Release\ekf_fusion.exe

# Run unit tests
.\bin\Release\ekf_tests.exe
```

### 4. Analyze Results

```bash
cd ..
python scripts/analyze_results.py
```

## Linux Build

### 1. Install Dependencies

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libgtest-dev \
    python3-pip

pip3 install pandas numpy matplotlib
```

### 2. Configure and Build

```bash
cd "Modular Multi-Sensor Fusion (Lidar + Radar) EKF"
mkdir -p build
cd build

cmake ..
cmake --build . --config Release -j$(nproc)
```

### 3. Run Simulation and Tests

```bash
# Run the main simulation
./bin/ekf_fusion

# Run unit tests
./bin/ekf_tests
```

### 4. Analyze Results

```bash
cd ..
python3 scripts/analyze_results.py
```

## macOS Build

### 1. Install Dependencies

```bash
# Using Homebrew
brew install cmake eigen googletest
pip3 install pandas numpy matplotlib
```

### 2. Configure and Build

```bash
cd "Modular Multi-Sensor Fusion (Lidar + Radar) EKF"
mkdir -p build
cd build

cmake ..
cmake --build . --config Release -j$(sysctl -n hw.ncpu)
```

### 3. Run Simulation and Tests

```bash
# Run the main simulation
./bin/ekf_fusion

# Run unit tests
./bin/ekf_tests
```

### 4. Analyze Results

```bash
cd ..
python3 scripts/analyze_results.py
```

## Typical Output

After running `./bin/ekf_fusion`, you should see:

```
=== Multi-Sensor Fusion EKF Simulation ===
Target Motion: Circular path
Lidar Frequency: 10 Hz
Radar Frequency: 20 Hz (offset 0.025s)

Filter initialized with LIDAR measurement
Initial state: [9.9801, 0.0897, 0, 0]

Simulation complete. Processed 1998 samples

=== Error Analysis ===
Position RMSE:
  Fused:       0.1234 m
  Lidar-only:  0.2156 m
  Radar-only:  0.3421 m

=== NIS Consistency Check ===
Lidar NIS count:  200
Lidar NIS avg:    1.9523
Radar NIS count:  400
Radar NIS avg:    3.0147

Results saved to fusion_results.csv
```

Then running `python scripts/analyze_results.py`:

```
======================================================================
Multi-Sensor Fusion EKF Analysis
======================================================================

Loaded 1998 samples
Simulation time: 0.00s to 20.00s

======================================================================
ERROR METRICS ANALYSIS
======================================================================

Position RMSE (meters):
  Fused:         0.1234
  Lidar-only:    0.2156
  Radar-only:    0.3421
  Improvement over Lidar-only: 42.8%
  Improvement over Radar-only: 63.9%

Position Mean Error (meters):
  Fused:         0.1021
  Lidar-only:    0.1832
  Radar-only:    0.2956

Position Max Error (meters):
  Fused:         0.4123
  Lidar-only:    0.7821
  Radar-only:    1.1245

Velocity RMSE (m/s):
  Fused:         0.3456
  Mean error:    0.2891

======================================================================
Generating visualization plots...
----------------------------------------------------------------------
Saved: 1_trajectories.png
Saved: 2_velocity.png
Saved: 3_error_distribution.png
Saved: 4_performance_comparison.png
Saved: 5_position_components.png
----------------------------------------------------------------------

Analysis complete! Generated 5 PNG files:
  1. 1_trajectories.png - XY trajectories and error over time
  2. 2_velocity.png - Velocity components and magnitude
  3. 3_error_distribution.png - Error histograms
  4. 4_performance_comparison.png - Performance bars
  5. 5_position_components.png - Position components and errors

======================================================================
```

## Troubleshooting

### CMake Configuration Errors

**Error: Could not find Eigen3**
```bash
# Find where Eigen3 is installed
find /usr -name "*Eigen3*" 2>/dev/null

# Pass the path to CMake
cmake .. -DEIGEN3_INCLUDE_DIR=/path/to/eigen3
```

**Error: Could not find GTest**
```bash
# On Ubuntu, after installing libgtest-dev, you need to build it
cd /usr/src/gtest
sudo cmake .
sudo make
sudo cp lib/*.a /usr/lib

# Then reconfigure
cmake ..
```

### Runtime Errors

**Segmentation fault during simulation**
- Ensure Eigen3 is correctly installed
- Check that all matrix sizes are correct (4x4, 2x4, etc.)

**NIS values look wrong**
- Verify measurement noise matrices match sensor specs
- Check process noise parameters (noise_ax, noise_ay)
- Ensure angle normalization is applied

## Next Steps

1. Review the [README.md](README.md) for comprehensive documentation
2. Examine the source code:
   - [src/fusion_ekf.h](src/fusion_ekf.h) - EKF class definition
   - [src/tools.cpp](src/tools.cpp) - Utility implementations
   - [src/main.cpp](src/main.cpp) - Simulation code
3. Run the unit tests: `./bin/ekf_tests` (Linux) or `./bin/ekf_tests.exe` (Windows)
4. Experiment with different parameters in the simulation
5. Extend with your own sensors or trajectories

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review the comments in the source code
3. Run tests in verbose mode: `./bin/ekf_tests --gtest_filter=*`
4. Check that all dependencies are installed: `cmake --version`, `g++ --version`, etc.
