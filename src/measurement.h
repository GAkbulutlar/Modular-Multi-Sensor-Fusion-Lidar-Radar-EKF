#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <stdint.h>

/**
 * @enum SensorType
 * @brief Identifies the sensor type for a measurement
 */
enum class SensorType : uint8_t {
    LIDAR,   ///< Lidar sensor (Cartesian coordinates: x, y)
    RADAR    ///< Radar sensor (Polar coordinates: rho, phi, rho_dot)
};

/**
 * @class MeasurementPackage
 * @brief Encapsulates a sensor measurement with metadata
 * 
 * Contains the sensor type, raw measurement values, and timestamp.
 * For Lidar: raw_measurements[0] = x, raw_measurements[1] = y
 * For Radar: raw_measurements[0] = rho, raw_measurements[1] = phi, raw_measurements[2] = rho_dot
 */
class MeasurementPackage {
public:
    SensorType sensor_type;           ///< Type of sensor (LIDAR or RADAR)
    long long timestamp;              ///< Timestamp in microseconds
    double raw_measurements[3];       ///< Raw measurement values (up to 3 values)
    
    /**
     * @brief Constructor for MeasurementPackage
     * @param type The sensor type
     * @param ts Timestamp in microseconds
     */
    MeasurementPackage(SensorType type, long long ts) 
        : sensor_type(type), timestamp(ts) {
        raw_measurements[0] = 0.0;
        raw_measurements[1] = 0.0;
        raw_measurements[2] = 0.0;
    }
};

#endif // MEASUREMENT_H
