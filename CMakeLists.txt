cmake_minimum_required(VERSION 3.16)
project(MultiSensorFusionEKF CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_USE_MATH_DEFINES)
# Find required packages
find_package(Eigen3 REQUIRED)

# Main executable
add_executable(ekf_fusion
    src/main.cpp
    src/fusion_ekf.cpp
    src/measurement.cpp
    src/tools.cpp
)

target_include_directories(ekf_fusion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(ekf_fusion PRIVATE Eigen3::Eigen)

# Set output directories
set_target_properties(ekf_fusion PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
