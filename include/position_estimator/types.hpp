#pragma once

#include <Eigen/Dense>
#include <cstdint>

namespace stampfly {
namespace estimator {

// Eigen type aliases for convenience
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;
using Matrix3d = Eigen::Matrix3d;
using MatrixXd = Eigen::MatrixXd;
using Quaterniond = Eigen::Quaterniond;

// State vector dimensions
constexpr int STATE_DIM = 15;          // Full state dimension
constexpr int ERROR_STATE_DIM = 15;    // Error state dimension

// State indices
constexpr int IDX_POS = 0;        // Position (3)
constexpr int IDX_VEL = 3;        // Velocity (3)
constexpr int IDX_QUAT = 6;       // Quaternion (4)
constexpr int IDX_GYRO_BIAS = 10; // Gyro bias (3)
constexpr int IDX_ACC_BIAS = 13;  // Accelerometer bias (2, z-axis separate)

// Error state indices (attitude is 3D rotation vector)
constexpr int ERR_IDX_POS = 0;        // δp (3)
constexpr int ERR_IDX_VEL = 3;        // δv (3)
constexpr int ERR_IDX_ATT = 6;        // δθ (3)
constexpr int ERR_IDX_GYRO_BIAS = 9;  // δb_g (3)
constexpr int ERR_IDX_ACC_BIAS = 12;  // δb_a (2)

/**
 * @brief IMU measurement data
 */
struct ImuData {
    double timestamp;           // Timestamp in seconds
    Vector3d acceleration;      // Specific force [m/s²] in body frame
    Vector3d angular_velocity;  // Angular velocity [rad/s] in body frame
};

/**
 * @brief Magnetometer measurement data
 */
struct MagnetometerData {
    double timestamp;           // Timestamp in seconds
    Vector3d magnetic_field;    // Magnetic field [unit vector or Gauss] in body frame
};

/**
 * @brief Barometer measurement data
 */
struct BarometerData {
    double timestamp;           // Timestamp in seconds
    double altitude;            // Altitude [m] above reference
};

/**
 * @brief Time-of-Flight (ToF) sensor measurement data
 */
struct TofData {
    double timestamp;           // Timestamp in seconds
    double distance;            // Distance [m] to ground
};

/**
 * @brief Optical flow measurement data
 */
struct OpticalFlowData {
    double timestamp;           // Timestamp in seconds
    Vector3d flow_rate;         // Flow rate [rad/s] in x and y, z unused
    double quality;             // Quality indicator [0-1]
};

/**
 * @brief Estimated state output
 */
struct EstimatorState {
    double timestamp;           // Timestamp in seconds

    // Position and velocity in world frame (NED)
    Vector3d position;          // [m]
    Vector3d velocity;          // [m/s]

    // Attitude
    Quaterniond quaternion;     // Orientation (body to world)
    Vector3d euler_angles;      // [roll, pitch, yaw] in radians

    // Biases in body frame
    Vector3d gyro_bias;         // [rad/s]
    Eigen::Vector2d acc_bias;   // [m/s²] (x, y only)

    // Covariance (diagonal elements for key states)
    Vector3d position_std;      // Position standard deviation [m]
    Vector3d velocity_std;      // Velocity standard deviation [m/s]
    Vector3d attitude_std;      // Attitude standard deviation [rad]
};

/**
 * @brief Coordinate system type
 */
enum class CoordinateSystem {
    NED,  // North-East-Down
    ENU   // East-North-Up (future support)
};

/**
 * @brief Estimator status
 */
enum class EstimatorStatus {
    NOT_INITIALIZED,
    INITIALIZING,
    RUNNING,
    DEGRADED,  // Running but with degraded accuracy
    ERROR
};

} // namespace estimator
} // namespace stampfly
