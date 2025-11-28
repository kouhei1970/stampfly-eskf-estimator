#include "position_estimator/filter_parameters.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>

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

    // Accelerometer attitude update (for roll/pitch correction)
    // Note: noise_std should be tuned based on actual sensor characteristics
    // Higher values = less aggressive attitude correction
    params.accelerometer_attitude.enabled = true;
    params.accelerometer_attitude.rate_hz = 50.0;
    params.accelerometer_attitude.noise_std = 0.5;  // Higher noise = less aggressive correction
    params.accelerometer_attitude.motion_threshold = 0.5;  // Reject during motion

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

    // Accelerometer attitude update (useful outdoors)
    params.accelerometer_attitude.enabled = true;
    params.accelerometer_attitude.rate_hz = 50.0;
    params.accelerometer_attitude.noise_std = 0.08;  // Lower noise outdoors
    params.accelerometer_attitude.motion_threshold = 0.3;  // Tighter threshold

    // Outlier rejection
    params.outlier_rejection.enabled = true;
    params.outlier_rejection.mahalanobis_threshold = 9.49;  // 95% for 3-DOF

    // Gravity
    params.gravity = 9.81;

    return params;
}

// ==================== Simple YAML Parser ====================
// Minimal YAML parser for configuration files.
// Supports: nested keys, numbers, booleans, strings, comments.
// Does NOT support: arrays, multi-line strings, anchors, aliases.

namespace {

// Trim whitespace from both ends of string
std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// Count leading spaces (for indentation)
int countIndent(const std::string& line) {
    int count = 0;
    for (char c : line) {
        if (c == ' ') count++;
        else if (c == '\t') count += 2;  // Treat tab as 2 spaces
        else break;
    }
    return count;
}

// Remove quotes from string value
std::string unquote(const std::string& s) {
    if (s.size() >= 2) {
        if ((s.front() == '"' && s.back() == '"') ||
            (s.front() == '\'' && s.back() == '\'')) {
            return s.substr(1, s.size() - 2);
        }
    }
    return s;
}

// Parse a simple value (bool, number, or string)
struct YamlValue {
    enum Type { NONE, BOOL, NUMBER, STRING };
    Type type = NONE;
    bool boolVal = false;
    double numVal = 0.0;
    std::string strVal;

    static YamlValue parse(const std::string& s) {
        YamlValue v;
        std::string val = trim(s);

        // Remove comment
        size_t commentPos = val.find('#');
        if (commentPos != std::string::npos) {
            val = trim(val.substr(0, commentPos));
        }

        if (val.empty()) {
            return v;
        }

        // Check for boolean
        std::string lower = val;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
        if (lower == "true" || lower == "yes" || lower == "on") {
            v.type = BOOL;
            v.boolVal = true;
            return v;
        }
        if (lower == "false" || lower == "no" || lower == "off") {
            v.type = BOOL;
            v.boolVal = false;
            return v;
        }

        // Check for number
        try {
            size_t pos;
            double d = std::stod(val, &pos);
            if (pos == val.size()) {
                v.type = NUMBER;
                v.numVal = d;
                return v;
            }
        } catch (...) {}

        // Default to string
        v.type = STRING;
        v.strVal = unquote(val);
        return v;
    }
};

// Simple hierarchical key-value store
class YamlConfig {
public:
    // Store values with hierarchical keys like "estimator.process_noise.sigma_a"
    std::map<std::string, YamlValue> values;

    bool load(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "ERROR: Cannot open YAML file: " << path << "\n";
            return false;
        }

        std::vector<std::pair<int, std::string>> keyStack;  // (indent, key)
        std::string line;

        while (std::getline(file, line)) {
            // Skip empty lines and comments
            std::string trimmed = trim(line);
            if (trimmed.empty() || trimmed[0] == '#') {
                continue;
            }

            int indent = countIndent(line);

            // Find colon separator
            size_t colonPos = trimmed.find(':');
            if (colonPos == std::string::npos) {
                continue;  // Invalid line, skip
            }

            std::string key = trim(trimmed.substr(0, colonPos));
            std::string valueStr = trim(trimmed.substr(colonPos + 1));

            // Pop keys with equal or greater indent
            while (!keyStack.empty() && keyStack.back().first >= indent) {
                keyStack.pop_back();
            }

            // Build full key path
            std::string fullKey;
            for (const auto& kv : keyStack) {
                if (!fullKey.empty()) fullKey += ".";
                fullKey += kv.second;
            }
            if (!fullKey.empty()) fullKey += ".";
            fullKey += key;

            // Parse value if present
            if (!valueStr.empty()) {
                YamlValue val = YamlValue::parse(valueStr);
                if (val.type != YamlValue::NONE) {
                    values[fullKey] = val;
                }
            }

            // Push current key for potential children
            keyStack.push_back({indent, key});
        }

        return true;
    }

    double getDouble(const std::string& key, double defaultVal = 0.0) const {
        auto it = values.find(key);
        if (it != values.end() && it->second.type == YamlValue::NUMBER) {
            return it->second.numVal;
        }
        return defaultVal;
    }

    bool getBool(const std::string& key, bool defaultVal = false) const {
        auto it = values.find(key);
        if (it != values.end() && it->second.type == YamlValue::BOOL) {
            return it->second.boolVal;
        }
        return defaultVal;
    }

    std::string getString(const std::string& key, const std::string& defaultVal = "") const {
        auto it = values.find(key);
        if (it != values.end() && it->second.type == YamlValue::STRING) {
            return it->second.strVal;
        }
        return defaultVal;
    }

    bool hasKey(const std::string& key) const {
        return values.find(key) != values.end();
    }
};

} // anonymous namespace

bool ESKFParameters::loadFromYAML(const std::string& yaml_path) {
    YamlConfig config;
    if (!config.load(yaml_path)) {
        return false;
    }

    // Start from indoor defaults
    *this = createIndoorDefault();

    // ========== Process Noise ==========
    if (config.hasKey("estimator.process_noise.sigma_a")) {
        process_noise.sigma_a = config.getDouble("estimator.process_noise.sigma_a");
    }
    if (config.hasKey("estimator.process_noise.sigma_omega")) {
        process_noise.sigma_omega = config.getDouble("estimator.process_noise.sigma_omega");
    }
    if (config.hasKey("estimator.process_noise.sigma_bg")) {
        process_noise.sigma_bg = config.getDouble("estimator.process_noise.sigma_bg");
    }
    if (config.hasKey("estimator.process_noise.sigma_ba")) {
        process_noise.sigma_ba = config.getDouble("estimator.process_noise.sigma_ba");
    }

    // ========== Initial Covariance ==========
    if (config.hasKey("estimator.initial_covariance.sigma_p")) {
        initial_covariance.sigma_p = config.getDouble("estimator.initial_covariance.sigma_p");
    }
    if (config.hasKey("estimator.initial_covariance.sigma_v")) {
        initial_covariance.sigma_v = config.getDouble("estimator.initial_covariance.sigma_v");
    }
    if (config.hasKey("estimator.initial_covariance.sigma_theta")) {
        initial_covariance.sigma_theta = config.getDouble("estimator.initial_covariance.sigma_theta");
    }
    if (config.hasKey("estimator.initial_covariance.sigma_bg")) {
        initial_covariance.sigma_bg = config.getDouble("estimator.initial_covariance.sigma_bg");
    }
    if (config.hasKey("estimator.initial_covariance.sigma_ba")) {
        initial_covariance.sigma_ba = config.getDouble("estimator.initial_covariance.sigma_ba");
    }

    // ========== IMU ==========
    if (config.hasKey("sensors.imu.rate_hz")) {
        imu.rate_hz = config.getDouble("sensors.imu.rate_hz");
    }
    if (config.hasKey("sensors.imu.accelerometer.noise_density")) {
        imu.accel_noise_density = config.getDouble("sensors.imu.accelerometer.noise_density");
    }
    if (config.hasKey("sensors.imu.accelerometer.bias_stability")) {
        imu.accel_bias_stability = config.getDouble("sensors.imu.accelerometer.bias_stability");
    }
    if (config.hasKey("sensors.imu.gyroscope.noise_density")) {
        imu.gyro_noise_density = config.getDouble("sensors.imu.gyroscope.noise_density");
    }
    if (config.hasKey("sensors.imu.gyroscope.bias_stability")) {
        imu.gyro_bias_stability = config.getDouble("sensors.imu.gyroscope.bias_stability");
    }

    // ========== Magnetometer ==========
    if (config.hasKey("sensors.magnetometer.enabled")) {
        magnetometer.enabled = config.getBool("sensors.magnetometer.enabled");
    }
    if (config.hasKey("sensors.magnetometer.rate_hz")) {
        magnetometer.rate_hz = config.getDouble("sensors.magnetometer.rate_hz");
    }
    if (config.hasKey("sensors.magnetometer.noise_std")) {
        magnetometer.noise_std = config.getDouble("sensors.magnetometer.noise_std");
    }
    if (config.hasKey("sensors.magnetometer.declination_deg")) {
        magnetometer.declination_deg = config.getDouble("sensors.magnetometer.declination_deg");
    }

    // ========== Barometer ==========
    if (config.hasKey("sensors.barometer.enabled")) {
        barometer.enabled = config.getBool("sensors.barometer.enabled");
    }
    if (config.hasKey("sensors.barometer.rate_hz")) {
        barometer.rate_hz = config.getDouble("sensors.barometer.rate_hz");
    }
    if (config.hasKey("sensors.barometer.noise_std")) {
        barometer.noise_std = config.getDouble("sensors.barometer.noise_std");
    }

    // ========== ToF ==========
    if (config.hasKey("sensors.tof.enabled")) {
        tof.enabled = config.getBool("sensors.tof.enabled");
    }
    if (config.hasKey("sensors.tof.rate_hz")) {
        tof.rate_hz = config.getDouble("sensors.tof.rate_hz");
    }
    if (config.hasKey("sensors.tof.noise_std")) {
        tof.noise_std = config.getDouble("sensors.tof.noise_std");
    }
    if (config.hasKey("sensors.tof.max_range")) {
        tof.max_range = config.getDouble("sensors.tof.max_range");
    }
    if (config.hasKey("sensors.tof.tilt_compensation")) {
        tof.tilt_compensation = config.getBool("sensors.tof.tilt_compensation");
    }

    // ========== Optical Flow ==========
    if (config.hasKey("sensors.optical_flow.enabled")) {
        optical_flow.enabled = config.getBool("sensors.optical_flow.enabled");
    }
    if (config.hasKey("sensors.optical_flow.rate_hz")) {
        optical_flow.rate_hz = config.getDouble("sensors.optical_flow.rate_hz");
    }
    if (config.hasKey("sensors.optical_flow.noise_std")) {
        optical_flow.noise_std = config.getDouble("sensors.optical_flow.noise_std");
    }
    if (config.hasKey("sensors.optical_flow.min_altitude")) {
        optical_flow.min_altitude = config.getDouble("sensors.optical_flow.min_altitude");
    }
    if (config.hasKey("sensors.optical_flow.max_altitude")) {
        optical_flow.max_altitude = config.getDouble("sensors.optical_flow.max_altitude");
    }
    if (config.hasKey("sensors.optical_flow.min_quality")) {
        optical_flow.min_quality = config.getDouble("sensors.optical_flow.min_quality");
    }

    // ========== Outlier Rejection ==========
    if (config.hasKey("outlier_rejection.enabled")) {
        outlier_rejection.enabled = config.getBool("outlier_rejection.enabled");
    }
    if (config.hasKey("outlier_rejection.mahalanobis_threshold")) {
        outlier_rejection.mahalanobis_threshold = config.getDouble("outlier_rejection.mahalanobis_threshold");
    }

    // ========== Gravity ==========
    if (config.hasKey("constants.gravity")) {
        gravity = config.getDouble("constants.gravity");
    }

    // ========== Coordinate System ==========
    if (config.hasKey("coordinate_system")) {
        std::string cs = config.getString("coordinate_system");
        if (cs == "NED") {
            coordinate_system = CoordinateSystem::NED;
        } else if (cs == "ENU") {
            coordinate_system = CoordinateSystem::ENU;
        }
    }

    // Validate loaded parameters
    if (!validate()) {
        std::cerr << "WARNING: Loaded YAML parameters failed validation\n";
        return false;
    }

    std::cout << "YAML parameters loaded from: " << yaml_path << "\n";
    return true;
}

} // namespace estimator
} // namespace stampfly
