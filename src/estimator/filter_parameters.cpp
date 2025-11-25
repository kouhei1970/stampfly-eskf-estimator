#include "position_estimator/filter_parameters.hpp"
#include <iostream>
#include <cmath>

namespace stampfly {
namespace estimator {

bool ESKFParameters::validate() const {
    bool valid = true;

    // Process noise parameters
    if (process_noise.sigma_a <= 0.0) {
        std::cerr << "ERROR: sigma_a must be positive\n";
        valid = false;
    }
    if (process_noise.sigma_omega <= 0.0) {
        std::cerr << "ERROR: sigma_omega must be positive\n";
        valid = false;
    }
    if (process_noise.sigma_bg <= 0.0) {
        std::cerr << "ERROR: sigma_bg must be positive\n";
        valid = false;
    }
    if (process_noise.sigma_ba <= 0.0) {
        std::cerr << "ERROR: sigma_ba must be positive\n";
        valid = false;
    }

    // Initial covariance parameters
    if (initial_covariance.sigma_p <= 0.0) {
        std::cerr << "ERROR: sigma_p must be positive\n";
        valid = false;
    }
    if (initial_covariance.sigma_v <= 0.0) {
        std::cerr << "ERROR: sigma_v must be positive\n";
        valid = false;
    }
    if (initial_covariance.sigma_theta <= 0.0) {
        std::cerr << "ERROR: sigma_theta must be positive\n";
        valid = false;
    }
    if (initial_covariance.sigma_bg <= 0.0) {
        std::cerr << "ERROR: sigma_bg (initial) must be positive\n";
        valid = false;
    }
    if (initial_covariance.sigma_ba <= 0.0) {
        std::cerr << "ERROR: sigma_ba (initial) must be positive\n";
        valid = false;
    }

    // IMU parameters
    if (imu.rate_hz <= 0.0) {
        std::cerr << "ERROR: IMU rate must be positive\n";
        valid = false;
    }
    if (imu.accel_noise_density <= 0.0) {
        std::cerr << "ERROR: Accel noise density must be positive\n";
        valid = false;
    }
    if (imu.gyro_noise_density <= 0.0) {
        std::cerr << "ERROR: Gyro noise density must be positive\n";
        valid = false;
    }

    // Magnetometer parameters
    if (magnetometer.enabled) {
        if (magnetometer.rate_hz <= 0.0) {
            std::cerr << "ERROR: Magnetometer rate must be positive\n";
            valid = false;
        }
        if (magnetometer.noise_std <= 0.0) {
            std::cerr << "ERROR: Magnetometer noise std must be positive\n";
            valid = false;
        }
    }

    // Barometer parameters
    if (barometer.enabled) {
        if (barometer.rate_hz <= 0.0) {
            std::cerr << "ERROR: Barometer rate must be positive\n";
            valid = false;
        }
        if (barometer.noise_std <= 0.0) {
            std::cerr << "ERROR: Barometer noise std must be positive\n";
            valid = false;
        }
    }

    // ToF parameters
    if (tof.enabled) {
        if (tof.rate_hz <= 0.0) {
            std::cerr << "ERROR: ToF rate must be positive\n";
            valid = false;
        }
        if (tof.noise_std <= 0.0) {
            std::cerr << "ERROR: ToF noise std must be positive\n";
            valid = false;
        }
        if (tof.max_range <= 0.0) {
            std::cerr << "ERROR: ToF max range must be positive\n";
            valid = false;
        }
    }

    // Optical flow parameters
    if (optical_flow.enabled) {
        if (optical_flow.rate_hz <= 0.0) {
            std::cerr << "ERROR: Optical flow rate must be positive\n";
            valid = false;
        }
        if (optical_flow.noise_std <= 0.0) {
            std::cerr << "ERROR: Optical flow noise std must be positive\n";
            valid = false;
        }
        if (optical_flow.min_altitude < 0.0) {
            std::cerr << "ERROR: Optical flow min altitude must be non-negative\n";
            valid = false;
        }
        if (optical_flow.max_altitude <= optical_flow.min_altitude) {
            std::cerr << "ERROR: Optical flow max altitude must be > min altitude\n";
            valid = false;
        }
        if (optical_flow.min_quality < 0.0 || optical_flow.min_quality > 1.0) {
            std::cerr << "ERROR: Optical flow min quality must be in [0, 1]\n";
            valid = false;
        }
    }

    // Outlier rejection parameters
    if (outlier_rejection.enabled) {
        if (outlier_rejection.mahalanobis_threshold <= 0.0) {
            std::cerr << "ERROR: Mahalanobis threshold must be positive\n";
            valid = false;
        }
        // Typical range check (chi-squared critical values)
        if (outlier_rejection.mahalanobis_threshold < 1.0 ||
            outlier_rejection.mahalanobis_threshold > 50.0) {
            std::cerr << "WARNING: Mahalanobis threshold seems unusual (typical: 5-15)\n";
        }
    }

    // Gravity check
    if (std::abs(gravity - 9.81) > 0.5) {
        std::cerr << "WARNING: Gravity value unusual (typical: 9.81 m/s²)\n";
    }

    return valid;
}

ESKFParameters ESKFParameters::createIndoorDefault() {
    ESKFParameters params;

    // Coordinate system
    params.coordinate_system = CoordinateSystem::NED;

    // Process noise (conservative for indoor)
    params.process_noise.sigma_a = 0.15;       // Higher accel noise indoors
    params.process_noise.sigma_omega = 0.002;  // Higher gyro noise
    params.process_noise.sigma_bg = 0.0001;    // Gyro bias random walk
    params.process_noise.sigma_ba = 0.002;     // Accel bias random walk

    // Initial covariance (conservative)
    params.initial_covariance.sigma_p = 0.5;      // 0.5m position uncertainty
    params.initial_covariance.sigma_v = 0.3;      // 0.3m/s velocity uncertainty
    params.initial_covariance.sigma_theta = 0.1;  // 0.1rad (~6deg) attitude uncertainty
    params.initial_covariance.sigma_bg = 0.01;    // Gyro bias uncertainty
    params.initial_covariance.sigma_ba = 0.1;     // Accel bias uncertainty

    // IMU (high rate)
    params.imu.rate_hz = 200.0;
    params.imu.accel_noise_density = 0.15;
    params.imu.accel_bias_stability = 0.002;
    params.imu.gyro_noise_density = 0.002;
    params.imu.gyro_bias_stability = 0.0001;

    // Magnetometer (enabled for indoor heading)
    params.magnetometer.enabled = true;
    params.magnetometer.rate_hz = 50.0;
    params.magnetometer.noise_std = 0.3;  // Higher noise due to indoor disturbances
    params.magnetometer.declination_deg = 0.0;

    // Barometer (enabled but less reliable indoors)
    params.barometer.enabled = true;
    params.barometer.rate_hz = 20.0;
    params.barometer.noise_std = 1.5;  // Higher noise indoors (HVAC, etc.)

    // ToF sensor (primary altitude source indoors)
    params.tof.enabled = true;
    params.tof.rate_hz = 50.0;
    params.tof.noise_std = 0.05;  // Low noise for good ToF sensor
    params.tof.max_range = 4.0;
    params.tof.tilt_compensation = true;  // Important for indoor maneuvers

    // Optical flow (primary velocity source indoors)
    params.optical_flow.enabled = true;
    params.optical_flow.rate_hz = 30.0;
    params.optical_flow.noise_std = 0.8;  // Moderate noise
    params.optical_flow.min_altitude = 0.2;  // Minimum useful altitude
    params.optical_flow.max_altitude = 3.0;  // Indoor ceiling height
    params.optical_flow.min_quality = 0.5;   // Moderate quality threshold

    // Outlier rejection (important for indoor)
    params.outlier_rejection.enabled = true;
    params.outlier_rejection.mahalanobis_threshold = 7.81;  // 95% for 2-DOF

    // Gravity
    params.gravity = 9.81;

    return params;
}

ESKFParameters ESKFParameters::createOutdoorDefault() {
    ESKFParameters params;

    // Coordinate system
    params.coordinate_system = CoordinateSystem::NED;

    // Process noise (tighter for outdoor)
    params.process_noise.sigma_a = 0.1;        // Lower accel noise outdoors
    params.process_noise.sigma_omega = 0.001;  // Lower gyro noise
    params.process_noise.sigma_bg = 0.00005;   // Gyro bias random walk
    params.process_noise.sigma_ba = 0.001;     // Accel bias random walk

    // Initial covariance (moderate)
    params.initial_covariance.sigma_p = 2.0;      // 2m position uncertainty (GPS-like)
    params.initial_covariance.sigma_v = 0.5;      // 0.5m/s velocity uncertainty
    params.initial_covariance.sigma_theta = 0.1;  // 0.1rad attitude uncertainty
    params.initial_covariance.sigma_bg = 0.01;    // Gyro bias uncertainty
    params.initial_covariance.sigma_ba = 0.1;     // Accel bias uncertainty

    // IMU (high rate)
    params.imu.rate_hz = 200.0;
    params.imu.accel_noise_density = 0.1;
    params.imu.accel_bias_stability = 0.001;
    params.imu.gyro_noise_density = 0.001;
    params.imu.gyro_bias_stability = 0.00005;

    // Magnetometer (enabled for heading)
    params.magnetometer.enabled = true;
    params.magnetometer.rate_hz = 50.0;
    params.magnetometer.noise_std = 0.2;  // Lower noise outdoors
    params.magnetometer.declination_deg = 0.0;  // Set based on location

    // Barometer (primary altitude source outdoors)
    params.barometer.enabled = true;
    params.barometer.rate_hz = 20.0;
    params.barometer.noise_std = 0.5;  // Lower noise outdoors

    // ToF sensor (limited range outdoors, disabled by default)
    params.tof.enabled = false;  // Typically too high for ToF range
    params.tof.rate_hz = 50.0;
    params.tof.noise_std = 0.05;
    params.tof.max_range = 4.0;
    params.tof.tilt_compensation = false;

    // Optical flow (less reliable outdoors due to altitude, disabled)
    params.optical_flow.enabled = false;  // Too high for optical flow
    params.optical_flow.rate_hz = 30.0;
    params.optical_flow.noise_std = 1.5;
    params.optical_flow.min_altitude = 0.5;
    params.optical_flow.max_altitude = 10.0;  // Higher ceiling outdoors
    params.optical_flow.min_quality = 0.6;    // Higher quality threshold

    // Outlier rejection
    params.outlier_rejection.enabled = true;
    params.outlier_rejection.mahalanobis_threshold = 9.49;  // 95% for 3-DOF

    // Gravity
    params.gravity = 9.81;

    return params;
}

bool ESKFParameters::loadFromYAML(const std::string& yaml_path) {
    // TODO: Implement YAML loading
    // For now, this is a placeholder that returns false
    std::cerr << "WARNING: YAML loading not yet implemented\n";
    std::cerr << "         Use createIndoorDefault() or createOutdoorDefault() instead\n";
    std::cerr << "         YAML path requested: " << yaml_path << "\n";
    return false;
}

} // namespace estimator
} // namespace stampfly
