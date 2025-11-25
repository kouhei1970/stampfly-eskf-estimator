#pragma once

#include "types.hpp"
#include "filter_parameters.hpp"
#include "sensor_interface.hpp"
#include <memory>

namespace stampfly {
namespace estimator {

/**
 * @brief Error-State Kalman Filter (ESKF) Position and Attitude Estimator
 *
 * This class implements a tightly-coupled ESKF for estimating the 3D position,
 * velocity, and attitude of a quadcopter using IMU, magnetometer, barometer,
 * ToF, and optical flow sensors.
 *
 * Key features:
 * - Error-state formulation for numerical stability
 * - Quaternion-based attitude representation
 * - Support for asynchronous sensor updates
 * - Outlier rejection using Mahalanobis distance
 * - Modular sensor interface
 *
 * Usage:
 * 1. Create estimator with parameters: ESKFEstimator estimator(params);
 * 2. Initialize with initial state: estimator.initialize(initial_state);
 * 3. Process sensor data:
 *    - IMU at high rate: estimator.processImu(imu_data);
 *    - Other sensors asynchronously: estimator.processMagnetometer(mag_data);
 * 4. Get estimated state: EstimatorState state = estimator.getState();
 *
 * Thread safety: This class is NOT thread-safe. External synchronization
 * is required if called from multiple threads.
 */
class ESKFEstimator {
public:
    /**
     * @brief Construct ESKF estimator with parameters
     * @param params Filter parameters (process noise, sensor noise, etc.)
     */
    explicit ESKFEstimator(const ESKFParameters& params);

    /**
     * @brief Destructor
     */
    ~ESKFEstimator();

    // Prevent copying (use shared_ptr if needed)
    ESKFEstimator(const ESKFEstimator&) = delete;
    ESKFEstimator& operator=(const ESKFEstimator&) = delete;

    /**
     * @brief Initialize the estimator with initial state
     * @param initial_state Initial position, velocity, and attitude
     * @return true if initialization successful
     */
    bool initialize(const EstimatorState& initial_state);

    /**
     * @brief Initialize the estimator with default hovering state
     * @return true if initialization successful
     */
    bool initializeDefault();

    /**
     * @brief Reset the estimator to initial state
     */
    void reset();

    /**
     * @brief Check if estimator is initialized and ready
     */
    bool isInitialized() const;

    /**
     * @brief Get current estimator status
     */
    EstimatorStatus getStatus() const;

    // ======================== Sensor Processing ========================

    /**
     * @brief Process IMU measurement (prediction step)
     *
     * This is the main prediction step and should be called at the IMU rate
     * (typically 100-200 Hz). It propagates the nominal state and error
     * covariance forward in time.
     *
     * @param imu IMU measurement data
     */
    void processImu(const ImuData& imu);

    /**
     * @brief Process magnetometer measurement (update step)
     * @param mag Magnetometer measurement data
     * @return true if update was accepted (not rejected as outlier)
     */
    bool processMagnetometer(const MagnetometerData& mag);

    /**
     * @brief Process barometer measurement (update step)
     * @param baro Barometer measurement data
     * @return true if update was accepted
     */
    bool processBarometer(const BarometerData& baro);

    /**
     * @brief Process ToF sensor measurement (update step)
     * @param tof ToF measurement data
     * @return true if update was accepted
     */
    bool processTof(const TofData& tof);

    /**
     * @brief Process optical flow measurement (update step)
     * @param flow Optical flow measurement data
     * @return true if update was accepted
     */
    bool processOpticalFlow(const OpticalFlowData& flow);

    // ======================== State Access ========================

    /**
     * @brief Get complete estimated state
     * @return Current estimator state with position, velocity, attitude, etc.
     */
    EstimatorState getState() const;

    /**
     * @brief Get position estimate in world frame
     * @return Position [x, y, z] in NED frame [m]
     */
    Vector3 getPosition() const;

    /**
     * @brief Get velocity estimate in world frame
     * @return Velocity [vx, vy, vz] in NED frame [m/s]
     */
    Vector3 getVelocity() const;

    /**
     * @brief Get attitude as quaternion
     * @return Quaternion representing body-to-world rotation
     */
    Quaternion getQuaternion() const;

    /**
     * @brief Get attitude as Euler angles
     * @return Euler angles [roll, pitch, yaw] in radians
     */
    Vector3 getEulerAngles() const;

    /**
     * @brief Get gyroscope bias estimate
     * @return Gyro bias [bx, by, bz] in body frame [rad/s]
     */
    Vector3 getGyroBias() const;

    /**
     * @brief Get accelerometer bias estimate
     * @return Accel bias [bx, by] in body frame [m/s²] (z component unused)
     */
    Vector3 getAccelBias() const;

    /**
     * @brief Get state covariance matrix
     * @return Error state covariance matrix (15x15)
     */
    Matrix getCovariance() const;

    // ======================== Sensor Suite Integration ========================

    /**
     * @brief Connect sensor suite to estimator
     *
     * This registers callbacks with the sensor suite so that sensor data
     * is automatically processed by the estimator.
     *
     * @param sensors Sensor suite to connect
     */
    void connectSensorSuite(std::shared_ptr<SensorSuite> sensors);

    /**
     * @brief Disconnect sensor suite
     */
    void disconnectSensorSuite();

    // ======================== Parameter Management ========================

    /**
     * @brief Update filter parameters (can be done during operation)
     * @param params New parameters
     */
    void updateParameters(const ESKFParameters& params);

    /**
     * @brief Get current filter parameters
     */
    const ESKFParameters& getParameters() const;

    // ======================== Diagnostics ========================

    /**
     * @brief Get time since last IMU measurement
     * @return Time in seconds
     */
    double getTimeSinceLastImu() const;

    /**
     * @brief Get number of rejected measurements (for diagnostics)
     * @return Count of rejected outlier measurements
     */
    uint32_t getRejectedMeasurementCount() const;

    /**
     * @brief Reset rejected measurement counter
     */
    void resetDiagnostics();

private:
    // Private implementation (PIMPL idiom to hide implementation details)
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace estimator
} // namespace stampfly
