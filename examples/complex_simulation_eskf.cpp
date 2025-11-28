/**
 * @file complex_simulation_eskf.cpp
 * @brief Complex simulation demonstrating ESKF estimator with flight maneuvers
 *
 * This example demonstrates:
 * 1. Takeoff from ground level
 * 2. Hover and stabilize
 * 3. Forward flight maneuver
 * 4. Yaw rotation
 * 5. Landing
 *
 * The simulation includes realistic sensor noise and sensor fusion.
 */

#include "position_estimator/eskf_estimator.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <vector>

using namespace stampfly::estimator;

// ========== Simulation Utilities ==========

// Simple pseudo-random noise generator (deterministic for reproducibility)
class NoiseGenerator {
public:
    NoiseGenerator(double seed = 12345.0) : state_(seed) {}

    double gaussian(double std) {
        // Box-Muller transform
        double u1 = uniform01();
        double u2 = uniform01();
        double z = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
        return z * std;
    }

private:
    double uniform01() {
        // Linear congruential generator
        state_ = std::fmod(state_ * 1103515245.0 + 12345.0, 2147483648.0);
        return state_ / 2147483648.0;
    }
    double state_;
};

// Ground truth state for comparison
struct GroundTruth {
    double time;
    Vector3 position;
    Vector3 velocity;
    Quaternion quaternion;
    Vector3 euler;
};

// Data record for logging
struct DataRecord {
    double time;
    int phase;
    // Ground truth
    double truth_px, truth_py, truth_pz;
    double truth_vx, truth_vy, truth_vz;
    double truth_roll, truth_pitch, truth_yaw;
    // Estimate
    double est_px, est_py, est_pz;
    double est_vx, est_vy, est_vz;
    double est_roll, est_pitch, est_yaw;
    // Standard deviations
    double std_px, std_py, std_pz;
    double std_vx, std_vy, std_vz;
    double std_roll, std_pitch, std_yaw;
    // Error
    double err_px, err_py, err_pz;
    double err_norm;
};

// Covariance record for heatmap visualization
struct CovarianceRecord {
    double time;
    int phase;
    std::vector<double> covariance;  // 15x15 = 225 elements (row-major)
};

// Save data to CSV file
void saveToCSV(const std::vector<DataRecord>& records, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot open file for writing: " << filename << "\n";
        return;
    }

    // Header
    file << "time,phase,"
         << "truth_px,truth_py,truth_pz,truth_vx,truth_vy,truth_vz,truth_roll,truth_pitch,truth_yaw,"
         << "est_px,est_py,est_pz,est_vx,est_vy,est_vz,est_roll,est_pitch,est_yaw,"
         << "std_px,std_py,std_pz,std_vx,std_vy,std_vz,std_roll,std_pitch,std_yaw,"
         << "err_px,err_py,err_pz,err_norm\n";

    // Data
    file << std::fixed << std::setprecision(6);
    for (const auto& r : records) {
        file << r.time << "," << r.phase << ","
             << r.truth_px << "," << r.truth_py << "," << r.truth_pz << ","
             << r.truth_vx << "," << r.truth_vy << "," << r.truth_vz << ","
             << r.truth_roll << "," << r.truth_pitch << "," << r.truth_yaw << ","
             << r.est_px << "," << r.est_py << "," << r.est_pz << ","
             << r.est_vx << "," << r.est_vy << "," << r.est_vz << ","
             << r.est_roll << "," << r.est_pitch << "," << r.est_yaw << ","
             << r.std_px << "," << r.std_py << "," << r.std_pz << ","
             << r.std_vx << "," << r.std_vy << "," << r.std_vz << ","
             << r.std_roll << "," << r.std_pitch << "," << r.std_yaw << ","
             << r.err_px << "," << r.err_py << "," << r.err_pz << "," << r.err_norm << "\n";
    }

    file.close();
    std::cout << "Data saved to: " << filename << " (" << records.size() << " records)\n";
}

// Save covariance matrix data to CSV file
void saveCovarianceToCSV(const std::vector<CovarianceRecord>& records, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot open file for writing: " << filename << "\n";
        return;
    }

    // Header: time, phase, and 225 covariance elements (P_0_0, P_0_1, ..., P_14_14)
    file << "time,phase";
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            file << ",P_" << i << "_" << j;
        }
    }
    file << "\n";

    // Data
    file << std::scientific << std::setprecision(8);
    for (const auto& r : records) {
        file << r.time << "," << r.phase;
        for (const auto& val : r.covariance) {
            file << "," << val;
        }
        file << "\n";
    }

    file.close();
    std::cout << "Covariance data saved to: " << filename << " (" << records.size() << " records)\n";
}

// ========== Flight Phase Definitions ==========

enum class FlightPhase {
    GROUND,
    TAKEOFF,
    HOVER_1,
    FORWARD,
    HOVER_2,
    YAW,
    LANDING,
    COMPLETE
};

const char* phaseToString(FlightPhase phase) {
    switch (phase) {
        case FlightPhase::GROUND: return "GROUND";
        case FlightPhase::TAKEOFF: return "TAKEOFF";
        case FlightPhase::HOVER_1: return "HOVER_1";
        case FlightPhase::FORWARD: return "FORWARD";
        case FlightPhase::HOVER_2: return "HOVER_2";
        case FlightPhase::YAW: return "YAW";
        case FlightPhase::LANDING: return "LANDING";
        case FlightPhase::COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
}

// ========== Main Simulation ==========

int main() {
    std::cout << "StampFly ESKF Estimator - Complex Simulation\n";
    std::cout << "=============================================\n\n";

    // ========== Initialize Parameters ==========
    // Load from YAML or use defaults
    ESKFParameters params;
    if (!params.loadFromYAML("../config/indoor_params.yaml")) {
        std::cout << "Using indoor default parameters (YAML not found)\n";
        params = ESKFParameters::createIndoorDefault();
    }

    // Reduce accelerometer attitude correction aggressiveness for maneuvers
    params.accelerometer_attitude.noise_std = 1.0;
    params.accelerometer_attitude.motion_threshold = 0.3;

    // Improve optical flow velocity tracking
    params.optical_flow.noise_std = 0.3;  // Lower noise = stronger velocity correction

    ESKFEstimator estimator(params);

    // ========== Initialize at Ground Level ==========
    EstimatorState initial_state;
    initial_state.timestamp = 0.0;
    initial_state.position = Vector3(0.0, 0.0, 0.0);  // On ground
    initial_state.velocity = Vector3(0.0, 0.0, 0.0);
    initial_state.quaternion = Quaternion::identity();

    estimator.initialize(initial_state);
    std::cout << "Estimator initialized at ground level\n\n";

    // ========== Simulation Setup ==========
    const double dt = 0.005;  // 200 Hz IMU
    const double sim_duration = 20.0;  // 20 seconds total
    NoiseGenerator noise(42.0);  // Reproducible noise

    // Flight plan timing (continuous, no gaps)
    const double takeoff_start = 0.5;
    const double takeoff_end = 3.0;
    const double hover1_end = 5.0;
    const double forward_start = 5.0;
    const double forward_end = 10.0;
    const double hover2_end = 12.0;
    const double yaw_start = 12.0;
    const double yaw_end = 14.0;
    const double hover3_end = 16.0;   // Added: hover before landing
    const double landing_start = 16.0;
    const double landing_end = 19.0;

    const double target_altitude = 1.5;  // meters (NED positive down)
    const double forward_velocity = 0.5; // m/s

    // Ground truth tracking
    GroundTruth truth;
    truth.position = Vector3::zero();
    truth.velocity = Vector3::zero();
    truth.quaternion = Quaternion::identity();
    truth.euler = Vector3::zero();

    FlightPhase current_phase = FlightPhase::GROUND;
    std::vector<std::pair<double, double>> position_errors;  // (time, error)
    std::vector<DataRecord> data_records;  // For CSV output
    std::vector<CovarianceRecord> covariance_records;  // For covariance heatmap

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Starting " << sim_duration << "s simulation...\n\n";

    // ========== Main Simulation Loop ==========
    double time = 0.0;
    FlightPhase prev_phase = FlightPhase::GROUND;

    while (time < sim_duration) {
        // ========== Determine Current Phase ==========
        if (time < takeoff_start) {
            current_phase = FlightPhase::GROUND;
        } else if (time < takeoff_end) {
            current_phase = FlightPhase::TAKEOFF;
        } else if (time < hover1_end) {
            current_phase = FlightPhase::HOVER_1;
        } else if (time < forward_end) {
            current_phase = FlightPhase::FORWARD;
        } else if (time < hover2_end) {
            current_phase = FlightPhase::HOVER_2;
        } else if (time < yaw_end) {
            current_phase = FlightPhase::YAW;
        } else if (time < hover3_end) {
            current_phase = FlightPhase::HOVER_2;  // Reuse HOVER_2 for pre-landing hover
        } else if (time < landing_end) {
            current_phase = FlightPhase::LANDING;
        } else {
            current_phase = FlightPhase::COMPLETE;
        }

        // Print phase transitions
        if (current_phase != prev_phase) {
            std::cout << "\n[" << time << "s] Phase: " << phaseToString(current_phase) << "\n";
            prev_phase = current_phase;
        }

        // ========== Update Ground Truth ==========
        Vector3 cmd_accel(0, 0, 0);  // Commanded acceleration
        Vector3 cmd_omega(0, 0, 0);  // Commanded angular velocity

        switch (current_phase) {
            case FlightPhase::GROUND:
                // Stay on ground
                truth.position = Vector3(0, 0, 0);
                truth.velocity = Vector3(0, 0, 0);
                break;

            case FlightPhase::TAKEOFF:
                // Smooth takeoff to target altitude
                {
                    double progress = (time - takeoff_start) / (takeoff_end - takeoff_start);
                    double z = target_altitude * (1 - std::cos(progress * M_PI)) / 2;
                    double vz = (M_PI / (takeoff_end - takeoff_start)) *
                                target_altitude * std::sin(progress * M_PI) / 2;
                    truth.position = Vector3(0, 0, z);
                    truth.velocity = Vector3(0, 0, vz);
                    cmd_accel = Vector3(0, 0, vz / dt);  // Simplified
                }
                break;

            case FlightPhase::HOVER_1:
                // Maintain position at target altitude
                truth.position = Vector3(truth.position.x(), truth.position.y(), target_altitude);
                truth.velocity = Vector3(0, 0, 0);
                cmd_accel = Vector3(0, 0, 0);
                break;

            case FlightPhase::HOVER_2:
                // Maintain current XY position at target altitude
                truth.position = Vector3(truth.position.x(), truth.position.y(), target_altitude);
                truth.velocity = Vector3(0, 0, 0);
                cmd_accel = Vector3(0, 0, 0);
                break;

            case FlightPhase::FORWARD:
                // Forward flight (positive X in NED) at constant altitude
                {
                    double progress = (time - forward_start) / (forward_end - forward_start);
                    // Smooth velocity ramp
                    double v_scale;
                    if (progress < 0.2) {
                        v_scale = progress / 0.2;  // Ramp up
                    } else if (progress > 0.8) {
                        v_scale = (1.0 - progress) / 0.2;  // Ramp down
                    } else {
                        v_scale = 1.0;  // Cruise
                    }

                    truth.velocity = Vector3(forward_velocity * v_scale, 0, 0);
                    // Update X position, maintain altitude
                    truth.position = Vector3(
                        truth.position.x() + truth.velocity.x() * dt,
                        truth.position.y(),
                        target_altitude  // Keep constant altitude
                    );
                }
                break;

            case FlightPhase::YAW:
                // Yaw rotation (90 degrees) while hovering
                {
                    double progress = (time - yaw_start) / (yaw_end - yaw_start);
                    double yaw = (M_PI / 2) * progress;  // 0 to 90 degrees
                    truth.euler = Vector3(0, 0, yaw);
                    truth.quaternion = Quaternion::fromEuler(0, 0, yaw);
                    truth.position = Vector3(truth.position.x(), truth.position.y(), target_altitude);
                    truth.velocity = Vector3(0, 0, 0);
                    cmd_omega = Vector3(0, 0, (M_PI / 2) / (yaw_end - yaw_start));
                }
                break;

            case FlightPhase::LANDING:
                // Smooth landing
                {
                    double progress = (time - landing_start) / (landing_end - landing_start);
                    double z = target_altitude * (1 - progress);
                    truth.position = Vector3(truth.position.x(), truth.position.y(), z);
                    truth.velocity = Vector3(0, 0, -target_altitude / (landing_end - landing_start));
                }
                break;

            case FlightPhase::COMPLETE:
                truth.position = Vector3(truth.position.x(), truth.position.y(), 0);
                truth.velocity = Vector3(0, 0, 0);
                break;
        }
        truth.time = time;

        // ========== Generate Sensor Data with Noise ==========

        // IMU (200 Hz)
        ImuData imu;
        imu.timestamp = time;

        // Use conjugate quaternion for world-to-body transformation (R^T)
        Quaternion q_conj = truth.quaternion.conjugate();

        // ========== IMU Specific Force Model ==========
        // The accelerometer measures "specific force" f = a - g (not acceleration a)
        //
        // In NED coordinate system:
        // - g_world = [0, 0, +9.81] (gravity points down, positive z)
        // - When hovering (a = 0): f_world = -g = [0, 0, -9.81]
        //
        // The ESKF prediction uses: v_dot = R * f_body + g_world
        // When hovering with identity rotation:
        //   v_dot = R * f_body + [0,0,9.81]
        //   f_body should be [0,0,-9.81] so v_dot = 0 (correct!)
        //
        // When tilted, f_body changes direction but not magnitude

        // For simplicity in this simulation, we assume small command accelerations
        // and compute specific force directly in body frame
        Vector3 f_body(0, 0, -9.81);  // Specific force when hovering (body frame)

        // Add noise
        imu.acceleration = Vector3(
            f_body.x() + noise.gaussian(0.01),
            f_body.y() + noise.gaussian(0.01),
            f_body.z() + noise.gaussian(0.01)
        );

        imu.angular_velocity = Vector3(
            cmd_omega.x() + noise.gaussian(0.001),
            cmd_omega.y() + noise.gaussian(0.001),
            cmd_omega.z() + noise.gaussian(0.001)
        );

        estimator.processImu(imu);

        // Magnetometer (50 Hz) - only in stable phases
        if (static_cast<int>(time / dt) % 4 == 0 &&
            current_phase != FlightPhase::YAW) {
            MagnetometerData mag;
            mag.timestamp = time;
            // Transform north vector to body frame using conjugate quaternion
            Vector3 north_world(1, 0, 0);
            Vector3 n_body = q_conj.rotate(north_world);
            mag.magnetic_field = Vector3(
                n_body.x() + noise.gaussian(0.05),
                n_body.y() + noise.gaussian(0.05),
                n_body.z() + noise.gaussian(0.05)
            );
            estimator.processMagnetometer(mag);
        }

        // ToF (50 Hz) - valid when altitude > 0.1m OR on ground (reading ~0)
        if (static_cast<int>(time / dt) % 4 == 0) {
            if (truth.position.z() > 0.1) {
                TofData tof;
                tof.timestamp = time;
                tof.distance = truth.position.z() + noise.gaussian(0.02);
                estimator.processTof(tof);
            } else if (current_phase == FlightPhase::GROUND ||
                       current_phase == FlightPhase::COMPLETE) {
                // On ground: ToF reads near-zero distance
                TofData tof;
                tof.timestamp = time;
                tof.distance = 0.05 + noise.gaussian(0.01);  // Small value for ground
                estimator.processTof(tof);
            }
        }

        // Barometer (20 Hz)
        if (static_cast<int>(time / dt) % 10 == 0) {
            BarometerData baro;
            baro.timestamp = time;
            baro.altitude = truth.position.z() + noise.gaussian(0.5);  // More noisy
            estimator.processBarometer(baro);
        }

        // Optical flow (30 Hz) - only valid at moderate altitude
        if (static_cast<int>(time / dt) % 7 == 0 &&
            truth.position.z() > 0.2 && truth.position.z() < 3.0) {
            OpticalFlowData flow;
            flow.timestamp = time;
            // Convert world velocity to body frame using conjugate quaternion
            Vector3 v_body = q_conj.rotate(truth.velocity);
            double h = truth.position.z();
            flow.flow_rate = Vector3(
                (v_body.x() / h) + noise.gaussian(0.1),
                (v_body.y() / h) + noise.gaussian(0.1),
                0
            );
            flow.quality = 0.9;
            estimator.processOpticalFlow(flow);
        }

        // Accelerometer attitude (50 Hz) - in stable phases (hovering, ground, complete)
        if (static_cast<int>(time / dt) % 4 == 0 &&
            (current_phase == FlightPhase::HOVER_1 ||
             current_phase == FlightPhase::HOVER_2 ||
             current_phase == FlightPhase::GROUND ||
             current_phase == FlightPhase::COMPLETE)) {
            estimator.processAccelerometer(imu.acceleration);
        }

        // ========== Record Data ==========
        if (static_cast<int>(time / dt) % 4 == 0) {  // Every 0.02s (50Hz)
            EstimatorState state = estimator.getState();
            Vector3 pos_error = state.position - truth.position;
            double error_norm = pos_error.norm();

            // Record for CSV output
            DataRecord rec;
            rec.time = time;
            rec.phase = static_cast<int>(current_phase);

            // Ground truth
            rec.truth_px = truth.position.x();
            rec.truth_py = truth.position.y();
            rec.truth_pz = truth.position.z();
            rec.truth_vx = truth.velocity.x();
            rec.truth_vy = truth.velocity.y();
            rec.truth_vz = truth.velocity.z();
            rec.truth_roll = truth.euler.x() * 180.0 / M_PI;
            rec.truth_pitch = truth.euler.y() * 180.0 / M_PI;
            rec.truth_yaw = truth.euler.z() * 180.0 / M_PI;

            // Estimate
            rec.est_px = state.position.x();
            rec.est_py = state.position.y();
            rec.est_pz = state.position.z();
            rec.est_vx = state.velocity.x();
            rec.est_vy = state.velocity.y();
            rec.est_vz = state.velocity.z();
            rec.est_roll = state.euler_angles.x() * 180.0 / M_PI;
            rec.est_pitch = state.euler_angles.y() * 180.0 / M_PI;
            rec.est_yaw = state.euler_angles.z() * 180.0 / M_PI;

            // Standard deviations
            rec.std_px = state.position_std.x();
            rec.std_py = state.position_std.y();
            rec.std_pz = state.position_std.z();
            rec.std_vx = state.velocity_std.x();
            rec.std_vy = state.velocity_std.y();
            rec.std_vz = state.velocity_std.z();
            rec.std_roll = state.attitude_std.x() * 180.0 / M_PI;
            rec.std_pitch = state.attitude_std.y() * 180.0 / M_PI;
            rec.std_yaw = state.attitude_std.z() * 180.0 / M_PI;

            // Error
            rec.err_px = pos_error.x();
            rec.err_py = pos_error.y();
            rec.err_pz = pos_error.z();
            rec.err_norm = error_norm;

            data_records.push_back(rec);
            position_errors.push_back({time, error_norm});

            // Record covariance matrix (at lower rate: every 0.1s for heatmap)
            if (static_cast<int>(time / dt) % 20 == 0) {
                CovarianceRecord cov_rec;
                cov_rec.time = time;
                cov_rec.phase = static_cast<int>(current_phase);
                Matrix P = estimator.getCovariance();
                cov_rec.covariance.reserve(225);
                for (int i = 0; i < 15; ++i) {
                    for (int j = 0; j < 15; ++j) {
                        cov_rec.covariance.push_back(P(i, j));
                    }
                }
                covariance_records.push_back(cov_rec);
            }
        }

        // ========== Print Periodic Status ==========
        if (static_cast<int>(time / dt) % 400 == 0) {  // Every 2s
            EstimatorState state = estimator.getState();

            std::cout << "\n--- t=" << time << "s [" << phaseToString(current_phase) << "] ---\n";
            std::cout << "Truth:    pos=[" << truth.position.x() << ", "
                      << truth.position.y() << ", " << truth.position.z() << "] m\n";
            std::cout << "Estimate: pos=[" << state.position.x() << ", "
                      << state.position.y() << ", " << state.position.z() << "] m\n";

            Vector3 pos_err = state.position - truth.position;
            std::cout << "Error:    [" << pos_err.x() << ", "
                      << pos_err.y() << ", " << pos_err.z() << "] m"
                      << " (norm=" << pos_err.norm() << ")\n";

            std::cout << "Pos std:  [" << state.position_std.x() << ", "
                      << state.position_std.y() << ", " << state.position_std.z() << "] m\n";
        }

        time += dt;
    }

    // ========== Final Report ==========
    std::cout << "\n\n========== Simulation Complete ==========\n\n";

    EstimatorState final_state = estimator.getState();

    std::cout << "Final Ground Truth:\n";
    std::cout << "  Position: [" << truth.position.x() << ", "
              << truth.position.y() << ", " << truth.position.z() << "] m\n";
    std::cout << "  Euler: [" << truth.euler.x() * 180/M_PI << ", "
              << truth.euler.y() * 180/M_PI << ", "
              << truth.euler.z() * 180/M_PI << "] deg\n";

    std::cout << "\nFinal Estimate:\n";
    std::cout << "  Position: [" << final_state.position.x() << ", "
              << final_state.position.y() << ", " << final_state.position.z() << "] m\n";
    std::cout << "  Velocity: [" << final_state.velocity.x() << ", "
              << final_state.velocity.y() << ", " << final_state.velocity.z() << "] m/s\n";
    std::cout << "  Euler: [" << final_state.euler_angles.x() * 180/M_PI << ", "
              << final_state.euler_angles.y() * 180/M_PI << ", "
              << final_state.euler_angles.z() * 180/M_PI << "] deg\n";

    // Position error statistics
    double max_error = 0.0;
    double sum_error = 0.0;
    for (const auto& e : position_errors) {
        if (e.second > max_error) max_error = e.second;
        sum_error += e.second;
    }
    double mean_error = sum_error / position_errors.size();

    std::cout << "\nPosition Error Statistics:\n";
    std::cout << "  Mean error: " << mean_error << " m\n";
    std::cout << "  Max error:  " << max_error << " m\n";

    std::cout << "\nEstimator Diagnostics:\n";
    std::cout << "  Rejected measurements: " << estimator.getRejectedMeasurementCount() << "\n";
    std::cout << "  Status: ";
    switch (estimator.getStatus()) {
        case EstimatorStatus::RUNNING: std::cout << "RUNNING\n"; break;
        case EstimatorStatus::DEGRADED: std::cout << "DEGRADED\n"; break;
        case EstimatorStatus::ERROR: std::cout << "ERROR\n"; break;
        default: std::cout << "UNKNOWN\n";
    }

    std::cout << "\nBias Estimates:\n";
    std::cout << "  Gyro bias: [" << final_state.gyro_bias.x() << ", "
              << final_state.gyro_bias.y() << ", " << final_state.gyro_bias.z() << "] rad/s\n";
    std::cout << "  Accel bias: [" << final_state.acc_bias.x() << ", "
              << final_state.acc_bias.y() << "] m/s²\n";

    // ========== Save Data to CSV ==========
    saveToCSV(data_records, "simulation_results.csv");
    saveCovarianceToCSV(covariance_records, "covariance_data.csv");

    std::cout << "\n========== Complex Simulation Complete ==========\n";
    std::cout << "\nTo visualize results, run:\n";
    std::cout << "  python3 ../scripts/visualize_eskf.py simulation_results.csv\n";
    std::cout << "  python3 ../scripts/visualize_covariance.py covariance_data.csv\n";

    return 0;
}
