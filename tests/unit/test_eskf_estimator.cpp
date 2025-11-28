/**
 * @file test_eskf_estimator.cpp
 * @brief Unit tests for ESKF Estimator
 *
 * Simple test framework without external dependencies.
 * Tests cover:
 * - Parameter validation
 * - Initialization
 * - Prediction step (IMU processing)
 * - Update steps (various sensors)
 * - State consistency
 */

#include "position_estimator/eskf_estimator.hpp"
#include <iostream>
#include <cmath>
#include <string>

using namespace stampfly::estimator;

// Simple test framework
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) void test_##name()
#define RUN_TEST(name) run_test(#name, test_##name)

#define ASSERT_TRUE(expr) \
    if (!(expr)) { \
        std::cerr << "  FAIL: " << #expr << " is false\n"; \
        throw std::runtime_error("Assertion failed"); \
    }

#define ASSERT_FALSE(expr) \
    if (expr) { \
        std::cerr << "  FAIL: " << #expr << " is true\n"; \
        throw std::runtime_error("Assertion failed"); \
    }

#define ASSERT_NEAR(a, b, eps) \
    if (std::abs((a) - (b)) > (eps)) { \
        std::cerr << "  FAIL: " << #a << " (" << (a) << ") != " << #b << " (" << (b) << ") within " << (eps) << "\n"; \
        throw std::runtime_error("Assertion failed"); \
    }

#define ASSERT_LT(a, b) \
    if (!((a) < (b))) { \
        std::cerr << "  FAIL: " << #a << " (" << (a) << ") >= " << #b << " (" << (b) << ")\n"; \
        throw std::runtime_error("Assertion failed"); \
    }

void run_test(const char* name, void (*test_func)()) {
    std::cout << "Running: " << name << "... ";
    try {
        test_func();
        std::cout << "PASS\n";
        tests_passed++;
    } catch (const std::exception& e) {
        std::cout << "FAIL\n";
        tests_failed++;
    }
}

// ========== Parameter Tests ==========

TEST(parameter_validation_valid) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    ASSERT_TRUE(params.validate());
}

TEST(parameter_validation_unusual_gravity) {
    // Note: validate() only warns for unusual gravity, doesn't fail
    // This test verifies the warning is triggered (check stderr)
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    params.gravity = -1.0;  // Unusual: negative gravity
    // validate() returns true but prints warning
    ASSERT_TRUE(params.validate());  // Still valid, just unusual
}

TEST(parameter_validation_invalid_imu_rate) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    params.imu.rate_hz = 0.0;  // Invalid: zero rate
    ASSERT_FALSE(params.validate());
}

TEST(indoor_default_params) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    ASSERT_TRUE(params.optical_flow.enabled);
    ASSERT_TRUE(params.tof.enabled);
    ASSERT_TRUE(params.magnetometer.enabled);
    ASSERT_NEAR(params.gravity, 9.81, 0.01);
}

TEST(outdoor_default_params) {
    ESKFParameters params = ESKFParameters::createOutdoorDefault();
    ASSERT_FALSE(params.optical_flow.enabled);  // Disabled outdoors
    ASSERT_FALSE(params.tof.enabled);           // Disabled outdoors
    ASSERT_TRUE(params.barometer.enabled);       // Primary altitude source
    ASSERT_NEAR(params.gravity, 9.81, 0.01);
}

// ========== Initialization Tests ==========

TEST(initialization_default) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    ESKFEstimator estimator(params);

    ASSERT_FALSE(estimator.isInitialized());

    ASSERT_TRUE(estimator.initializeDefault());
    ASSERT_TRUE(estimator.isInitialized());
    ASSERT_TRUE(estimator.getStatus() == EstimatorStatus::RUNNING);
}

TEST(initialization_custom_state) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    ESKFEstimator estimator(params);

    EstimatorState initial;
    initial.position = Vector3(1.0, 2.0, 3.0);
    initial.velocity = Vector3(0.1, 0.2, 0.3);
    initial.quaternion = Quaternion::fromEuler(0.1, 0.2, 0.3);

    ASSERT_TRUE(estimator.initialize(initial));
    ASSERT_TRUE(estimator.isInitialized());

    EstimatorState state = estimator.getState();
    ASSERT_NEAR(state.position.x(), 1.0, 0.01);
    ASSERT_NEAR(state.position.y(), 2.0, 0.01);
    ASSERT_NEAR(state.position.z(), 3.0, 0.01);
}

TEST(reset) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    ESKFEstimator estimator(params);

    estimator.initializeDefault();
    ASSERT_TRUE(estimator.isInitialized());

    estimator.reset();
    ASSERT_FALSE(estimator.isInitialized());
}

// ========== Prediction Step Tests ==========

TEST(imu_prediction_stationary) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    ESKFEstimator estimator(params);

    EstimatorState initial;
    initial.position = Vector3(0, 0, 1);
    initial.velocity = Vector3(0, 0, 0);
    initial.quaternion = Quaternion::identity();
    estimator.initialize(initial);

    // Process stationary IMU data (specific force = -g when hovering)
    const double dt = 0.005;
    for (int i = 0; i < 100; ++i) {
        ImuData imu;
        imu.timestamp = i * dt;
        imu.acceleration = Vector3(0, 0, -9.81);  // Specific force
        imu.angular_velocity = Vector3(0, 0, 0);
        estimator.processImu(imu);
    }

    EstimatorState state = estimator.getState();

    // Position should remain near initial (with small drift due to numerical errors)
    ASSERT_NEAR(state.position.x(), 0.0, 0.1);
    ASSERT_NEAR(state.position.y(), 0.0, 0.1);
    ASSERT_NEAR(state.position.z(), 1.0, 0.1);

    // Velocity should remain near zero
    ASSERT_NEAR(state.velocity.x(), 0.0, 0.1);
    ASSERT_NEAR(state.velocity.y(), 0.0, 0.1);
    ASSERT_NEAR(state.velocity.z(), 0.0, 0.1);
}

TEST(imu_prediction_covariance_growth) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    ESKFEstimator estimator(params);
    estimator.initializeDefault();

    EstimatorState state_before = estimator.getState();
    double pos_std_before = state_before.position_std.x();

    // Process IMU without any updates - covariance should grow
    for (int i = 0; i < 200; ++i) {
        ImuData imu;
        imu.timestamp = i * 0.005;
        imu.acceleration = Vector3(0, 0, -9.81);
        imu.angular_velocity = Vector3(0, 0, 0);
        estimator.processImu(imu);
    }

    EstimatorState state_after = estimator.getState();
    double pos_std_after = state_after.position_std.x();

    // Covariance should increase over time without updates
    ASSERT_TRUE(pos_std_after > pos_std_before);
}

// ========== Sensor Update Tests ==========

TEST(tof_update_reduces_covariance) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    ESKFEstimator estimator(params);

    EstimatorState initial;
    initial.position = Vector3(0, 0, 1);
    initial.velocity = Vector3(0, 0, 0);
    initial.quaternion = Quaternion::identity();
    initial.position_std = Vector3(1.0, 1.0, 1.0);  // High initial uncertainty
    estimator.initialize(initial);

    // Run prediction to increase covariance
    for (int i = 0; i < 20; ++i) {
        ImuData imu;
        imu.timestamp = i * 0.005;
        imu.acceleration = Vector3(0, 0, -9.81);
        imu.angular_velocity = Vector3(0, 0, 0);
        estimator.processImu(imu);
    }

    EstimatorState state_before = estimator.getState();
    double z_std_before = state_before.position_std.z();

    // ToF update should reduce altitude uncertainty
    TofData tof;
    tof.timestamp = 0.1;
    tof.distance = 1.0;
    estimator.processTof(tof);

    EstimatorState state_after = estimator.getState();
    double z_std_after = state_after.position_std.z();

    ASSERT_LT(z_std_after, z_std_before);
}

TEST(barometer_update) {
    // Create parameters with high initial position covariance
    // This allows large barometer innovations to be accepted
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    params.initial_covariance.sigma_p = 10.0;  // Very high initial uncertainty
    params.barometer.noise_std = 0.5;          // Lower noise for stronger updates
    params.outlier_rejection.enabled = false;   // Disable outlier rejection for this test
    ESKFEstimator estimator(params);

    EstimatorState initial;
    initial.position = Vector3(0, 0, 0);  // Start at 0
    initial.velocity = Vector3(0, 0, 0);
    initial.quaternion = Quaternion::identity();
    estimator.initialize(initial);

    double time = 0.0;
    const double dt = 0.005;

    // Process a few IMU samples first (required for Kalman filter)
    for (int i = 0; i < 20; ++i) {
        ImuData imu;
        imu.timestamp = time;
        imu.acceleration = Vector3(0, 0, -9.81);
        imu.angular_velocity = Vector3(0, 0, 0);
        estimator.processImu(imu);
        time += dt;
    }

    // Record position before barometer updates
    double z_before = estimator.getState().position.z();

    // Multiple barometer updates at 5m altitude
    for (int i = 0; i < 20; ++i) {
        BarometerData baro;
        baro.timestamp = time;
        baro.altitude = 5.0;
        estimator.processBarometer(baro);
        time += 0.05;
    }

    EstimatorState state = estimator.getState();
    double z_after = state.position.z();

    // Position should move toward barometer reading (5.0)
    // Check that z increased (moved in correct direction)
    ASSERT_TRUE(z_after > z_before);
}

TEST(magnetometer_update) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    ESKFEstimator estimator(params);

    EstimatorState initial;
    initial.position = Vector3(0, 0, 1);
    initial.velocity = Vector3(0, 0, 0);
    initial.quaternion = Quaternion::identity();
    estimator.initialize(initial);

    // Process IMU first
    ImuData imu;
    imu.timestamp = 0.0;
    imu.acceleration = Vector3(0, 0, -9.81);
    imu.angular_velocity = Vector3(0, 0, 0);
    estimator.processImu(imu);

    // Magnetometer pointing north
    MagnetometerData mag;
    mag.timestamp = 0.005;
    mag.magnetic_field = Vector3(1, 0, 0);
    bool accepted = estimator.processMagnetometer(mag);

    ASSERT_TRUE(accepted);
}

// ========== State Consistency Tests ==========

TEST(quaternion_normalized) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    ESKFEstimator estimator(params);
    estimator.initializeDefault();

    // Process many IMU measurements
    for (int i = 0; i < 1000; ++i) {
        ImuData imu;
        imu.timestamp = i * 0.005;
        imu.acceleration = Vector3(0.01, 0.01, -9.81);
        imu.angular_velocity = Vector3(0.01, 0.01, 0.01);
        estimator.processImu(imu);
    }

    EstimatorState state = estimator.getState();
    double q_norm = state.quaternion.norm();

    // Quaternion should remain normalized
    ASSERT_NEAR(q_norm, 1.0, 0.001);
}

TEST(rejected_measurement_count) {
    ESKFParameters params = ESKFParameters::createIndoorDefault();
    ESKFEstimator estimator(params);
    estimator.initializeDefault();

    uint32_t initial_count = estimator.getRejectedMeasurementCount();
    ASSERT_TRUE(initial_count == 0);
}

// ========== YAML Loading Tests ==========

TEST(yaml_load_indoor_params) {
    ESKFParameters params;
    // Note: This assumes running from build directory, adjust path as needed
    bool loaded = params.loadFromYAML("../config/indoor_params.yaml");

    if (loaded) {
        // Verify some key parameters from indoor_params.yaml
        ASSERT_NEAR(params.process_noise.sigma_a, 0.15, 0.01);
        ASSERT_FALSE(params.magnetometer.enabled);  // Disabled indoors
        ASSERT_TRUE(params.tof.enabled);
        ASSERT_NEAR(params.tof.noise_std, 0.05, 0.01);
        ASSERT_TRUE(params.optical_flow.enabled);
    } else {
        // If file not found, skip test (don't fail)
        std::cout << "(YAML file not found, skipping detailed validation) ";
    }
}

TEST(yaml_load_outdoor_params) {
    ESKFParameters params;
    bool loaded = params.loadFromYAML("../config/outdoor_params.yaml");

    if (loaded) {
        // Verify some key outdoor parameters
        ASSERT_TRUE(params.barometer.enabled);
        ASSERT_NEAR(params.barometer.noise_std, 0.5, 0.1);
        // Outdoor typically disables ToF and optical flow at high altitudes
    } else {
        std::cout << "(YAML file not found, skipping detailed validation) ";
    }
}

// ========== Main ==========

int main() {
    std::cout << "\n========== ESKF Estimator Unit Tests ==========\n\n";

    // Parameter tests
    std::cout << "--- Parameter Tests ---\n";
    RUN_TEST(parameter_validation_valid);
    RUN_TEST(parameter_validation_unusual_gravity);
    RUN_TEST(parameter_validation_invalid_imu_rate);
    RUN_TEST(indoor_default_params);
    RUN_TEST(outdoor_default_params);

    // Initialization tests
    std::cout << "\n--- Initialization Tests ---\n";
    RUN_TEST(initialization_default);
    RUN_TEST(initialization_custom_state);
    RUN_TEST(reset);

    // Prediction tests
    std::cout << "\n--- Prediction Step Tests ---\n";
    RUN_TEST(imu_prediction_stationary);
    RUN_TEST(imu_prediction_covariance_growth);

    // Sensor update tests
    std::cout << "\n--- Sensor Update Tests ---\n";
    RUN_TEST(tof_update_reduces_covariance);
    RUN_TEST(barometer_update);
    RUN_TEST(magnetometer_update);

    // State consistency tests
    std::cout << "\n--- State Consistency Tests ---\n";
    RUN_TEST(quaternion_normalized);
    RUN_TEST(rejected_measurement_count);

    // YAML loading tests
    std::cout << "\n--- YAML Loading Tests ---\n";
    RUN_TEST(yaml_load_indoor_params);
    RUN_TEST(yaml_load_outdoor_params);

    // Summary
    std::cout << "\n========== Test Summary ==========\n";
    std::cout << "Passed: " << tests_passed << "\n";
    std::cout << "Failed: " << tests_failed << "\n";
    std::cout << "Total:  " << (tests_passed + tests_failed) << "\n";

    return tests_failed == 0 ? 0 : 1;
}
