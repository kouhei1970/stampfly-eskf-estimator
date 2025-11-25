/**
 * @file simple_estimation_eskf.cpp
 * @brief Simple example demonstrating ESKF estimator usage
 *
 * This example shows how to:
 * 1. Create and initialize an ESKF estimator
 * 2. Process simulated sensor data
 * 3. Retrieve and display estimated state
 *
 * This is a minimal example for learning purposes.
 * For production use, integrate with actual sensor hardware.
 */

#include "position_estimator/eskf_estimator.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace stampfly::estimator;

int main() {
    std::cout << "StampFly ESKF Estimator - Simple Example\n";
    std::cout << "=========================================\n\n";

    // ========== 1. Create filter parameters ==========
    ESKFParameters params = ESKFParameters::createIndoorDefault();

    std::cout << "Using indoor default parameters:\n";
    std::cout << "  - Optical flow: " << (params.optical_flow.enabled ? "enabled" : "disabled") << "\n";
    std::cout << "  - Magnetometer: " << (params.magnetometer.enabled ? "enabled" : "disabled") << "\n";
    std::cout << "  - ToF sensor: " << (params.tof.enabled ? "enabled" : "disabled") << "\n";
    std::cout << "  - Barometer: " << (params.barometer.enabled ? "enabled" : "disabled") << "\n\n";

    // ========== 2. Create estimator ==========
    ESKFEstimator estimator(params);

    // ========== 3. Initialize with hovering state ==========
    std::cout << "Initializing estimator with default hovering state...\n";
    if (!estimator.initializeDefault()) {
        std::cerr << "ERROR: Failed to initialize estimator\n";
        return 1;
    }
    std::cout << "Estimator initialized successfully\n\n";

    // ========== 4. Simulation parameters ==========
    const double dt = 0.005;  // 200 Hz IMU rate
    const double duration = 5.0;  // 5 seconds simulation
    const int num_steps = static_cast<int>(duration / dt);

    // ========== 5. Simulated sensor data ==========
    // Hovering drone with small disturbances

    std::cout << "Starting " << duration << "s simulation at " << 1.0/dt << " Hz...\n";
    std::cout << std::fixed << std::setprecision(3);

    for (int i = 0; i < num_steps; ++i) {
        double time = i * dt;

        // Create simulated IMU data (hovering with small noise)
        ImuData imu;
        imu.timestamp = time;

        // Hovering: acceleration ~ gravity (with small noise)
        imu.acceleration = Vector3(0.01 * std::sin(time),
                                   0.01 * std::cos(time),
                                   9.81);

        // Hovering: minimal angular velocity
        imu.angular_velocity = Vector3(0.001 * std::sin(time * 2),
                                      0.001 * std::cos(time * 2),
                                      0.0);

        // Process IMU (prediction step)
        estimator.processImu(imu);

        // Process other sensors at lower rates

        // Magnetometer at 50 Hz
        if (i % 4 == 0) {
            MagnetometerData mag;
            mag.timestamp = time;
            // Pointing north in NED frame (with noise)
            mag.magnetic_field = Vector3(1.0, 0.0, 0.0);
            mag.magnetic_field.normalize();
            estimator.processMagnetometer(mag);
        }

        // Barometer and ToF at 20 Hz
        if (i % 10 == 0) {
            BarometerData baro;
            baro.timestamp = time;
            baro.altitude = 0.0;  // Hovering at reference altitude
            estimator.processBarometer(baro);

            TofData tof;
            tof.timestamp = time;
            tof.distance = 1.0;  // 1m above ground
            estimator.processTof(tof);
        }

        // Optical flow at 30 Hz
        if (i % 7 == 0) {
            OpticalFlowData flow;
            flow.timestamp = time;
            flow.flow_rate = Vector3(0.0, 0.0, 0.0);  // No movement
            flow.quality = 1.0;
            estimator.processOpticalFlow(flow);
        }

        // Print state every 1 second
        if (i % 200 == 0) {
            EstimatorState state = estimator.getState();
            std::cout << "\n--- Time: " << time << " s ---\n";
            std::cout << "Position [m]:  ["
                      << state.position.x() << ", "
                      << state.position.y() << ", "
                      << state.position.z() << "]\n";
            std::cout << "Velocity [m/s]: ["
                      << state.velocity.x() << ", "
                      << state.velocity.y() << ", "
                      << state.velocity.z() << "]\n";
            std::cout << "Euler [deg]:   ["
                      << state.euler_angles.x() * 180.0 / M_PI << ", "
                      << state.euler_angles.y() * 180.0 / M_PI << ", "
                      << state.euler_angles.z() * 180.0 / M_PI << "]\n";
            std::cout << "Pos Std [m]:   ["
                      << state.position_std.x() << ", "
                      << state.position_std.y() << ", "
                      << state.position_std.z() << "]\n";
        }
    }

    // ========== 6. Final state ==========
    std::cout << "\n========== Final State ==========\n";
    EstimatorState final_state = estimator.getState();

    std::cout << "Position:  ["
              << final_state.position.x() << ", "
              << final_state.position.y() << ", "
              << final_state.position.z() << "] m\n";
    std::cout << "Velocity:  ["
              << final_state.velocity.x() << ", "
              << final_state.velocity.y() << ", "
              << final_state.velocity.z() << "] m/s\n";
    std::cout << "Attitude:  ["
              << final_state.euler_angles.x() * 180.0 / M_PI << ", "
              << final_state.euler_angles.y() * 180.0 / M_PI << ", "
              << final_state.euler_angles.z() * 180.0 / M_PI << "] deg\n";

    std::cout << "\nGyro bias: ["
              << final_state.gyro_bias.x() << ", "
              << final_state.gyro_bias.y() << ", "
              << final_state.gyro_bias.z() << "] rad/s\n";
    std::cout << "Accel bias: ["
              << final_state.acc_bias.x() << ", "
              << final_state.acc_bias.y() << "] m/s²\n";

    std::cout << "\nEstimator status: ";
    switch (estimator.getStatus()) {
        case EstimatorStatus::RUNNING: std::cout << "RUNNING\n"; break;
        case EstimatorStatus::DEGRADED: std::cout << "DEGRADED\n"; break;
        case EstimatorStatus::ERROR: std::cout << "ERROR\n"; break;
        default: std::cout << "UNKNOWN\n";
    }

    std::cout << "Rejected measurements: " << estimator.getRejectedMeasurementCount() << "\n";

    std::cout << "\n========== Example Complete ==========\n";

    return 0;
}
