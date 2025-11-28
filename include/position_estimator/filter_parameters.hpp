#pragma once

#include "types.hpp"
#include <string>

namespace stampfly {
namespace estimator {

/**
 * @brief Process noise parameters for ESKF
 */
struct ProcessNoiseParams {
    double sigma_a = 0.1;          // Acceleration noise [m/s²]
    double sigma_omega = 0.001;    // Angular velocity noise [rad/s]
    double sigma_bg = 0.00005;     // Gyro bias random walk [rad/s/√s]
    double sigma_ba = 0.001;       // Accel bias random walk [m/s²/√s]
};

/**
 * @brief Initial state covariance parameters
 */
struct InitialCovarianceParams {
    double sigma_p = 1.0;          // Initial position uncertainty [m]
    double sigma_v = 0.5;          // Initial velocity uncertainty [m/s]
    double sigma_theta = 0.1;      // Initial attitude uncertainty [rad]
    double sigma_bg = 0.01;        // Initial gyro bias uncertainty [rad/s]
    double sigma_ba = 0.1;         // Initial accel bias uncertainty [m/s²]
};

/**
 * @brief IMU sensor parameters
 */
struct ImuParams {
    double rate_hz = 200.0;
    double accel_noise_density = 0.1;      // [m/s²]
    double accel_bias_stability = 0.001;   // [m/s²]
    double gyro_noise_density = 0.001;     // [rad/s]
    double gyro_bias_stability = 0.00005;  // [rad/s]
};

/**
 * @brief Magnetometer sensor parameters
 */
struct MagnetometerParams {
    bool enabled = true;
    double rate_hz = 50.0;
    double noise_std = 0.3;
    double declination_deg = 0.0;  // Magnetic declination for location
};

/**
 * @brief Barometer sensor parameters
 */
struct BarometerParams {
    bool enabled = true;
    double rate_hz = 20.0;
    double noise_std = 1.0;  // [m]
};

/**
 * @brief ToF sensor parameters
 */
struct TofParams {
    bool enabled = true;
    double rate_hz = 50.0;
    double noise_std = 0.05;         // [m]
    double max_range = 4.0;          // [m]
    bool tilt_compensation = true;
};

/**
 * @brief Optical flow sensor parameters
 */
struct OpticalFlowParams {
    bool enabled = true;
    double rate_hz = 30.0;
    double noise_std = 1.0;          // [rad/s]
    double min_altitude = 0.1;       // [m]
    double max_altitude = 3.0;       // [m]
    double min_quality = 0.5;        // Minimum quality threshold
};

/**
 * @brief Accelerometer attitude update parameters
 *
 * Uses accelerometer to observe gravity direction and correct roll/pitch.
 * Should be used at lower rate than IMU to avoid over-correction during motion.
 */
struct AccelerometerAttitudeParams {
    bool enabled = true;
    double rate_hz = 50.0;           // Update rate [Hz]
    double noise_std = 0.1;          // Measurement noise [m/s²]
    double motion_threshold = 0.5;   // Reject if |a| differs from g by this much [m/s²]
};

/**
 * @brief Outlier rejection parameters
 */
struct OutlierRejectionParams {
    bool enabled = true;
    double mahalanobis_threshold = 7.81;  // 95% for 2-DOF
};

/**
 * @brief Complete ESKF filter parameters
 */
struct ESKFParameters {
    // Estimator configuration
    CoordinateSystem coordinate_system = CoordinateSystem::NED;

    // Process and measurement noise
    ProcessNoiseParams process_noise;
    InitialCovarianceParams initial_covariance;

    // Sensor parameters
    ImuParams imu;
    MagnetometerParams magnetometer;
    BarometerParams barometer;
    TofParams tof;
    OpticalFlowParams optical_flow;
    AccelerometerAttitudeParams accelerometer_attitude;

    // Outlier rejection
    OutlierRejectionParams outlier_rejection;

    // System constants
    double gravity = 9.81;  // [m/s²]

    /**
     * @brief Load parameters from YAML file
     * @param yaml_path Path to YAML configuration file
     * @return true if loaded successfully
     */
    bool loadFromYAML(const std::string& yaml_path);

    /**
     * @brief Validate parameter values
     * @return true if all parameters are valid
     */
    bool validate() const;

    /**
     * @brief Create default parameters for indoor flight
     */
    static ESKFParameters createIndoorDefault();

    /**
     * @brief Create default parameters for outdoor flight
     */
    static ESKFParameters createOutdoorDefault();
};

} // namespace estimator
} // namespace stampfly
