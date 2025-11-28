#include "position_estimator/eskf_estimator.hpp"
#include "stampfly_math/math_utils.hpp"
#include <cmath>
#include <iostream>

namespace stampfly {
namespace estimator {

using namespace stampfly::math;

// ==================== Private Implementation Class ====================

class ESKFEstimator::Impl {
public:
    // Constructor
    explicit Impl(const ESKFParameters& params);

    // Initialization
    bool initialize(const EstimatorState& initial_state);
    bool initializeDefault();
    void reset();

    // State access
    bool isInitialized() const { return initialized_; }
    EstimatorStatus getStatus() const { return status_; }

    EstimatorState getState() const;
    Vector3 getPosition() const { return position_; }
    Vector3 getVelocity() const { return velocity_; }
    Quaternion getQuaternion() const { return quaternion_; }
    Vector3 getEulerAngles() const { return quaternion_.toEulerAngles(); }
    Vector3 getGyroBias() const { return gyro_bias_; }
    Vector3 getAccelBias() const { return accel_bias_; }
    Matrix getCovariance() const { return P_; }

    // Sensor processing
    void processImu(const ImuData& imu);
    bool processMagnetometer(const MagnetometerData& mag);
    bool processBarometer(const BarometerData& baro);
    bool processTof(const TofData& tof);
    bool processOpticalFlow(const OpticalFlowData& flow);
    bool processAccelerometer(const Vector3& acceleration);

    // Parameter management
    void updateParameters(const ESKFParameters& params);
    const ESKFParameters& getParameters() const { return params_; }

    // Diagnostics
    double getTimeSinceLastImu() const;
    uint32_t getRejectedMeasurementCount() const { return rejected_count_; }
    void resetDiagnostics() { rejected_count_ = 0; }

private:
    // Parameters
    ESKFParameters params_;

    // Nominal state (15D)
    Vector3 position_;       // Position [m] in world frame
    Vector3 velocity_;       // Velocity [m/s] in world frame
    Quaternion quaternion_;  // Attitude (body to world)
    Vector3 gyro_bias_;      // Gyro bias [rad/s]
    Vector3 accel_bias_;     // Accel bias [m/s²] (z component unused)

    // Error state covariance (15×15)
    Matrix P_;

    // Constants
    Vector3 gravity_;         // Gravity vector [0, 0, 9.81] in NED
    Vector3 magnetic_field_;  // Magnetic field vector [m_N, 0, m_D]

    // State management
    bool initialized_;
    EstimatorStatus status_;
    double last_imu_time_;
    uint32_t rejected_count_;

    // Cached IMU data for optical flow
    Vector3 last_angular_velocity_;

    // Helper methods (to be implemented in future phases)
    void initializeCovariance();
    void propagateNominalState(const ImuData& imu, double dt);
    Matrix computeErrorStateJacobian(const Vector3& a_corrected,
                                      const Vector3& w_corrected,
                                      double dt) const;
    Matrix computeProcessNoise(double dt) const;
    void kalmanUpdate(const Matrix& z_meas, const Matrix& z_pred,
                      const Matrix& H, const Matrix& R);
    void injectErrorState(const Matrix& dx_error);
    bool checkMahalanobisDistance(const Matrix& innovation,
                                    const Matrix& S) const;
};

// ==================== Implementation ====================

ESKFEstimator::Impl::Impl(const ESKFParameters& params)
    : params_(params),
      position_(Vector3::zero()),
      velocity_(Vector3::zero()),
      quaternion_(Quaternion::identity()),
      gyro_bias_(Vector3::zero()),
      accel_bias_(Vector3::zero()),
      P_(Matrix::identity(ERROR_STATE_DIM)),
      gravity_(Vector3(0.0, 0.0, params.gravity)),
      magnetic_field_(Vector3(1.0, 0.0, 0.0)),  // Default: pointing north
      initialized_(false),
      status_(EstimatorStatus::NOT_INITIALIZED),
      last_imu_time_(-1.0),
      rejected_count_(0),
      last_angular_velocity_(Vector3::zero())
{
    // Validate parameters
    if (!params_.validate()) {
        std::cerr << "WARNING: Invalid parameters provided to ESKFEstimator\n";
    }

    // Compute magnetic field from declination
    double decl_rad = params_.magnetometer.declination_deg * M_PI / 180.0;
    double m_N = std::cos(decl_rad);
    double m_E = std::sin(decl_rad);
    magnetic_field_ = Vector3(m_N, m_E, 0.0);
    magnetic_field_.normalize();
}

bool ESKFEstimator::Impl::initialize(const EstimatorState& initial_state) {
    // Set nominal state from initial state
    position_ = initial_state.position;
    velocity_ = initial_state.velocity;
    quaternion_ = initial_state.quaternion;
    quaternion_.normalize();  // Ensure normalized

    gyro_bias_ = initial_state.gyro_bias;
    accel_bias_ = initial_state.acc_bias;

    // Initialize error state covariance
    initializeCovariance();

    // Reset diagnostics
    last_imu_time_ = initial_state.timestamp;
    rejected_count_ = 0;

    // Update status
    initialized_ = true;
    status_ = EstimatorStatus::RUNNING;

    std::cout << "ESKF initialized with custom state\n";
    std::cout << "  Position: [" << position_.x() << ", " << position_.y()
              << ", " << position_.z() << "] m\n";
    std::cout << "  Attitude: [" << getEulerAngles().x() * 180.0 / M_PI << ", "
              << getEulerAngles().y() * 180.0 / M_PI << ", "
              << getEulerAngles().z() * 180.0 / M_PI << "] deg\n";

    return true;
}

bool ESKFEstimator::Impl::initializeDefault() {
    // Default hovering state
    EstimatorState default_state;
    default_state.timestamp = 0.0;
    default_state.position = Vector3::zero();
    default_state.velocity = Vector3::zero();
    default_state.quaternion = Quaternion::identity();
    default_state.euler_angles = Vector3::zero();
    default_state.gyro_bias = Vector3::zero();
    default_state.acc_bias = Vector3::zero();
    default_state.position_std = Vector3::zero();
    default_state.velocity_std = Vector3::zero();
    default_state.attitude_std = Vector3::zero();

    return initialize(default_state);
}

void ESKFEstimator::Impl::reset() {
    position_ = Vector3::zero();
    velocity_ = Vector3::zero();
    quaternion_ = Quaternion::identity();
    gyro_bias_ = Vector3::zero();
    accel_bias_ = Vector3::zero();

    P_ = Matrix::identity(ERROR_STATE_DIM);
    initializeCovariance();

    initialized_ = false;
    status_ = EstimatorStatus::NOT_INITIALIZED;
    last_imu_time_ = -1.0;
    rejected_count_ = 0;

    std::cout << "ESKF reset to initial state\n";
}

void ESKFEstimator::Impl::initializeCovariance() {
    // Initialize error state covariance matrix P (15×15)
    P_ = Matrix::zeros(ERROR_STATE_DIM, ERROR_STATE_DIM);

    const auto& ic = params_.initial_covariance;

    // Position covariance (indices 0-2)
    for (int i = 0; i < 3; ++i) {
        P_(ERR_IDX_POS + i, ERR_IDX_POS + i) = ic.sigma_p * ic.sigma_p;
    }

    // Velocity covariance (indices 3-5)
    for (int i = 0; i < 3; ++i) {
        P_(ERR_IDX_VEL + i, ERR_IDX_VEL + i) = ic.sigma_v * ic.sigma_v;
    }

    // Attitude covariance (indices 6-8)
    for (int i = 0; i < 3; ++i) {
        P_(ERR_IDX_ATT + i, ERR_IDX_ATT + i) = ic.sigma_theta * ic.sigma_theta;
    }

    // Gyro bias covariance (indices 9-11)
    for (int i = 0; i < 3; ++i) {
        P_(ERR_IDX_GYRO_BIAS + i, ERR_IDX_GYRO_BIAS + i) = ic.sigma_bg * ic.sigma_bg;
    }

    // Accel bias covariance (indices 12-13)
    for (int i = 0; i < 2; ++i) {
        P_(ERR_IDX_ACC_BIAS + i, ERR_IDX_ACC_BIAS + i) = ic.sigma_ba * ic.sigma_ba;
    }

    // Ensure symmetry
    P_ = utils::enforceCovarianceProperties(P_, 1e-9);
}

EstimatorState ESKFEstimator::Impl::getState() const {
    EstimatorState state;

    state.timestamp = last_imu_time_;
    state.position = position_;
    state.velocity = velocity_;
    state.quaternion = quaternion_;
    state.euler_angles = quaternion_.toEulerAngles();
    state.gyro_bias = gyro_bias_;
    state.acc_bias = accel_bias_;

    // Compute standard deviations from covariance diagonal
    state.position_std = Vector3(
        std::sqrt(std::max(0.0, P_(ERR_IDX_POS + 0, ERR_IDX_POS + 0))),
        std::sqrt(std::max(0.0, P_(ERR_IDX_POS + 1, ERR_IDX_POS + 1))),
        std::sqrt(std::max(0.0, P_(ERR_IDX_POS + 2, ERR_IDX_POS + 2)))
    );

    state.velocity_std = Vector3(
        std::sqrt(std::max(0.0, P_(ERR_IDX_VEL + 0, ERR_IDX_VEL + 0))),
        std::sqrt(std::max(0.0, P_(ERR_IDX_VEL + 1, ERR_IDX_VEL + 1))),
        std::sqrt(std::max(0.0, P_(ERR_IDX_VEL + 2, ERR_IDX_VEL + 2)))
    );

    state.attitude_std = Vector3(
        std::sqrt(std::max(0.0, P_(ERR_IDX_ATT + 0, ERR_IDX_ATT + 0))),
        std::sqrt(std::max(0.0, P_(ERR_IDX_ATT + 1, ERR_IDX_ATT + 1))),
        std::sqrt(std::max(0.0, P_(ERR_IDX_ATT + 2, ERR_IDX_ATT + 2)))
    );

    return state;
}

void ESKFEstimator::Impl::updateParameters(const ESKFParameters& params) {
    params_ = params;
    if (!params_.validate()) {
        std::cerr << "WARNING: Invalid parameters provided to updateParameters()\n";
    }
}

double ESKFEstimator::Impl::getTimeSinceLastImu() const {
    if (last_imu_time_ < 0.0) {
        return -1.0;
    }
    // This would require storing current time; for now return 0
    return 0.0;
}

// ==================== Prediction Step (Phase 4) ====================

void ESKFEstimator::Impl::processImu(const ImuData& imu) {
    if (!initialized_) {
        std::cerr << "ERROR: processImu() called before initialization\n";
        return;
    }

    // Calculate dt
    double dt;
    if (last_imu_time_ < 0.0) {
        // First IMU measurement
        dt = 1.0 / params_.imu.rate_hz;
    } else {
        dt = imu.timestamp - last_imu_time_;
    }

    // Validate dt
    if (dt <= 0.0 || dt > 0.1) {
        // Invalid dt: skip this measurement
        std::cerr << "WARNING: Invalid dt = " << dt << " s, skipping\n";
        last_imu_time_ = imu.timestamp;
        return;
    }

    // Store last angular velocity for optical flow (needed later)
    last_angular_velocity_ = imu.angular_velocity;

    // Step 1: Propagate nominal state
    propagateNominalState(imu, dt);

    // Step 2: Propagate error state covariance
    // Bias-corrected values
    Vector3 b_a_3d(accel_bias_.x(), accel_bias_.y(), 0.0);
    Vector3 a_corrected = imu.acceleration - b_a_3d;
    Vector3 w_corrected = imu.angular_velocity - gyro_bias_;

    // Compute continuous-time error state Jacobian F_c
    Matrix F_d = computeErrorStateJacobian(a_corrected, w_corrected, dt);

    // Compute discrete-time process noise Q_d
    Matrix Q_d = computeProcessNoise(dt);

    // Propagate covariance: P = F_d * P * F_d^T + Q_d
    P_ = F_d * P_ * F_d.transpose() + Q_d;

    // Enforce symmetry and positive definiteness
    P_ = utils::enforceCovarianceProperties(P_, 1e-9);

    // Update timestamp
    last_imu_time_ = imu.timestamp;
}

bool ESKFEstimator::Impl::processMagnetometer(const MagnetometerData& mag) {
    if (!initialized_) {
        std::cerr << "ERROR: processMagnetometer() called before initialization\n";
        return false;
    }

    if (!params_.magnetometer.enabled) {
        return false;
    }

    // ========== Magnetometer Update (Phase 6) ==========
    //
    // Observation model:
    //   z_mag = R(q)^T * m_W + n_mag
    //
    // where m_W is the magnetic field in world frame (NED)
    //
    // This provides heading (yaw) observability

    // Get rotation matrix from current quaternion
    Matrix R = utils::quaternionToRotationMatrix(quaternion_);

    // Predicted measurement: z_pred = R^T * m_W
    // Convert Vector3 to column matrix for matrix multiplication
    Matrix m_W_mat(3, 1);
    m_W_mat(0, 0) = magnetic_field_.x();
    m_W_mat(1, 0) = magnetic_field_.y();
    m_W_mat(2, 0) = magnetic_field_.z();
    Matrix z_pred_mat = R.transpose() * m_W_mat;
    Vector3 z_pred_vec(z_pred_mat(0, 0), z_pred_mat(1, 0), z_pred_mat(2, 0));

    // Actual measurement
    Vector3 z_meas_vec = mag.magnetic_field;

    // Normalize measurements for better comparison
    z_pred_vec.normalize();
    z_meas_vec.normalize();

    // Convert to column matrices for Kalman update
    Matrix z_meas(3, 1);
    z_meas(0, 0) = z_meas_vec.x();
    z_meas(1, 0) = z_meas_vec.y();
    z_meas(2, 0) = z_meas_vec.z();

    Matrix z_pred(3, 1);
    z_pred(0, 0) = z_pred_vec.x();
    z_pred(1, 0) = z_pred_vec.y();
    z_pred(2, 0) = z_pred_vec.z();

    // ========== Compute Observation Jacobian H_mag (3×15) ==========
    // H_mag = [0, 0, -R^T * [m_W]×, 0, 0]
    Matrix H = Matrix::zeros(3, ERROR_STATE_DIM);

    // δh/δθ = -R^T * [m_W]×
    Matrix skew_m = utils::skewSymmetric(magnetic_field_);
    Matrix R_T_skew_m = R.transpose() * skew_m;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            H(i, ERR_IDX_ATT + j) = -R_T_skew_m(i, j);
        }
    }

    // Observation noise covariance
    double sigma_mag = params_.magnetometer.noise_std;
    Matrix R_noise = Matrix::identity(3) * (sigma_mag * sigma_mag);

    // Perform Kalman update
    kalmanUpdate(z_meas, z_pred, H, R_noise);

    return true;
}

bool ESKFEstimator::Impl::processBarometer(const BarometerData& baro) {
    if (!initialized_) {
        std::cerr << "ERROR: processBarometer() called before initialization\n";
        return false;
    }

    if (!params_.barometer.enabled) {
        return false;
    }

    // ========== Barometer Update (Phase 7) ==========
    //
    // Observation model:
    //   z_baro = p_z + n_baro
    //
    // Simple altitude measurement

    // Predicted measurement: z_pred = p_z
    double z_pred_scalar = position_.z();

    // Actual measurement
    double z_meas_scalar = baro.altitude;

    // Convert to column matrices
    Matrix z_meas(1, 1);
    z_meas(0, 0) = z_meas_scalar;

    Matrix z_pred(1, 1);
    z_pred(0, 0) = z_pred_scalar;

    // ========== Compute Observation Jacobian H_baro (1×15) ==========
    // H_baro = [0, 0, 1, 0, ..., 0]
    Matrix H = Matrix::zeros(1, ERROR_STATE_DIM);
    H(0, ERR_IDX_POS + 2) = 1.0;  // p_z

    // Observation noise covariance
    double sigma_baro = params_.barometer.noise_std;
    Matrix R_noise(1, 1);
    R_noise(0, 0) = sigma_baro * sigma_baro;

    // Perform Kalman update
    kalmanUpdate(z_meas, z_pred, H, R_noise);

    return true;
}

bool ESKFEstimator::Impl::processTof(const TofData& tof) {
    if (!initialized_) {
        std::cerr << "ERROR: processTof() called before initialization\n";
        return false;
    }

    if (!params_.tof.enabled) {
        return false;
    }

    // Check range validity
    if (tof.distance < 0.0 || tof.distance > params_.tof.max_range) {
        return false;  // Invalid measurement
    }

    // ========== ToF Update (Phase 7) ==========
    //
    // Observation model:
    //   With tilt compensation: z_tof = p_z / R(2,2)
    //   Without compensation:   z_tof = p_z

    Matrix R = utils::quaternionToRotationMatrix(quaternion_);
    double R_22 = R(2, 2);

    double z_pred_scalar;
    Matrix H = Matrix::zeros(1, ERROR_STATE_DIM);

    if (params_.tof.tilt_compensation) {
        // Tilt-compensated ToF
        // Check if drone is too tilted
        if (std::abs(R_22) < 0.1) {
            return false;  // Too much tilt, skip measurement
        }

        z_pred_scalar = position_.z() / R_22;

        // Jacobian (simplified, ignoring attitude dependency for numerical stability)
        H(0, ERR_IDX_POS + 2) = 1.0 / R_22;
    } else {
        // Simple ToF (small tilt approximation)
        z_pred_scalar = position_.z();
        H(0, ERR_IDX_POS + 2) = 1.0;
    }

    // Convert to column matrices
    Matrix z_meas(1, 1);
    z_meas(0, 0) = tof.distance;

    Matrix z_pred(1, 1);
    z_pred(0, 0) = z_pred_scalar;

    // Observation noise covariance
    double sigma_tof = params_.tof.noise_std;
    Matrix R_noise(1, 1);
    R_noise(0, 0) = sigma_tof * sigma_tof;

    // Perform Kalman update
    kalmanUpdate(z_meas, z_pred, H, R_noise);

    return true;
}

bool ESKFEstimator::Impl::processOpticalFlow(const OpticalFlowData& flow) {
    if (!initialized_) {
        std::cerr << "ERROR: processOpticalFlow() called before initialization\n";
        return false;
    }

    if (!params_.optical_flow.enabled) {
        return false;
    }

    // Check altitude validity
    double altitude = position_.z();
    if (altitude < params_.optical_flow.min_altitude ||
        altitude > params_.optical_flow.max_altitude) {
        return false;  // Altitude out of range
    }

    // Check quality
    if (flow.quality < params_.optical_flow.min_quality) {
        return false;  // Quality too low
    }

    // ========== Optical Flow Update (Phase 8) ==========
    //
    // Observation model:
    //   flow_x = (v_Bx - ω_By * h) / h
    //   flow_y = (v_By + ω_Bx * h) / h
    //
    // where v_B is body-frame velocity, ω is angular velocity, h is altitude

    // Get rotation matrix and body-frame velocity
    Matrix R = utils::quaternionToRotationMatrix(quaternion_);
    // Convert velocity to column matrix for multiplication
    Matrix v_mat(3, 1);
    v_mat(0, 0) = velocity_.x();
    v_mat(1, 0) = velocity_.y();
    v_mat(2, 0) = velocity_.z();
    Matrix v_B_mat = R.transpose() * v_mat;
    Vector3 v_B(v_B_mat(0, 0), v_B_mat(1, 0), v_B_mat(2, 0));

    // Bias-corrected angular velocity
    Vector3 w_corrected = last_angular_velocity_ - gyro_bias_;

    double h = altitude;

    // Predicted measurement
    double flow_x_pred = (v_B.x() - w_corrected.y() * h) / h;
    double flow_y_pred = (v_B.y() + w_corrected.x() * h) / h;

    // Convert to column matrices
    Matrix z_meas(2, 1);
    z_meas(0, 0) = flow.flow_rate.x();
    z_meas(1, 0) = flow.flow_rate.y();

    Matrix z_pred(2, 1);
    z_pred(0, 0) = flow_x_pred;
    z_pred(1, 0) = flow_y_pred;

    // ========== Compute Observation Jacobian H_flow (2×15) ==========
    Matrix H = Matrix::zeros(2, ERROR_STATE_DIM);

    // δh/δp_z = -(v_Bx - ω_By*h) / h² = -flow_x / h (for x)
    //         = -(v_By + ω_Bx*h) / h² = -flow_y / h (for y)
    H(0, ERR_IDX_POS + 2) = -flow_x_pred / h;
    H(1, ERR_IDX_POS + 2) = -flow_y_pred / h;

    // δh/δv = (1/h) * R^T (first two rows, first two columns affect flow_x, flow_y)
    Matrix R_T = R.transpose();
    for (int j = 0; j < 3; ++j) {
        H(0, ERR_IDX_VEL + j) = R_T(0, j) / h;
        H(1, ERR_IDX_VEL + j) = R_T(1, j) / h;
    }

    // δh/δθ: velocity in body frame depends on attitude
    // δv_B/δθ = -[v_B]× (skew symmetric of body velocity)
    Matrix skew_vB = utils::skewSymmetric(v_B);
    for (int j = 0; j < 3; ++j) {
        H(0, ERR_IDX_ATT + j) = -skew_vB(0, j) / h;
        H(1, ERR_IDX_ATT + j) = -skew_vB(1, j) / h;
    }

    // δh/δb_g: gyro bias affects angular velocity terms
    // flow_x depends on -ω_y, flow_y depends on +ω_x
    H(0, ERR_IDX_GYRO_BIAS + 1) = 1.0;   // b_gy affects flow_x
    H(1, ERR_IDX_GYRO_BIAS + 0) = -1.0;  // b_gx affects flow_y

    // Observation noise covariance
    double sigma_flow = params_.optical_flow.noise_std;
    Matrix R_noise = Matrix::identity(2) * (sigma_flow * sigma_flow);

    // Perform Kalman update
    kalmanUpdate(z_meas, z_pred, H, R_noise);

    return true;
}

bool ESKFEstimator::Impl::processAccelerometer(const Vector3& acceleration) {
    if (!initialized_) {
        std::cerr << "ERROR: processAccelerometer() called before initialization\n";
        return false;
    }

    if (!params_.accelerometer_attitude.enabled) {
        return false;
    }

    // ========== Accelerometer Attitude Update ==========
    //
    // Observation model:
    //   z_acc = R(q)^T * g_W + b_a^3d + n_acc
    //
    // In NED frame: g_W = [0, 0, g]^T
    // When hovering, accelerometer reads [0, 0, -g] (specific force)
    //
    // This provides roll and pitch observability (not yaw)

    // Motion rejection: skip update if acceleration magnitude differs significantly from g
    // Note: We compare the absolute magnitude since specific force direction varies with orientation
    double acc_norm = acceleration.norm();
    double motion_threshold = params_.accelerometer_attitude.motion_threshold;
    double acc_deviation = std::abs(acc_norm - params_.gravity);
    if (acc_deviation > motion_threshold) {
        // Too much linear acceleration (dynamic motion), skip update
        // This check ensures we only use accelerometer when hovering/quasi-static
        // Debug: uncomment to see rejections
        // std::cerr << "Accel attitude: motion rejected (|a|=" << acc_norm << ", g=" << params_.gravity << ")\n";
        return false;
    }

    // Get rotation matrix from current quaternion
    Matrix R = utils::quaternionToRotationMatrix(quaternion_);

    // Predicted measurement: z_pred = -R^T * g_W + b_a^3d
    // The accelerometer measures specific force f = a - g
    // When stationary (a = 0): f = -g, so z_pred = -R^T * g_W
    // Convert gravity to column matrix
    Matrix g_mat(3, 1);
    g_mat(0, 0) = gravity_.x();
    g_mat(1, 0) = gravity_.y();
    g_mat(2, 0) = gravity_.z();
    Matrix z_pred_mat = R.transpose() * g_mat * (-1.0);  // Negative for specific force

    // Add bias
    Vector3 b_a_3d(accel_bias_.x(), accel_bias_.y(), 0.0);
    Vector3 z_pred_vec(z_pred_mat(0, 0) + b_a_3d.x(),
                       z_pred_mat(1, 0) + b_a_3d.y(),
                       z_pred_mat(2, 0) + b_a_3d.z());

    // Actual measurement
    Vector3 z_meas_vec = acceleration;

    // Convert to column matrices for Kalman update
    Matrix z_meas(3, 1);
    z_meas(0, 0) = z_meas_vec.x();
    z_meas(1, 0) = z_meas_vec.y();
    z_meas(2, 0) = z_meas_vec.z();

    Matrix z_pred(3, 1);
    z_pred(0, 0) = z_pred_vec.x();
    z_pred(1, 0) = z_pred_vec.y();
    z_pred(2, 0) = z_pred_vec.z();

    // ========== Compute Observation Jacobian H_acc (3×15) ==========
    // For z_pred = -R^T * g_W + b_a:
    // H_acc = [0, 0, +R^T * [g_W]×, 0, I_{3×2}]
    //
    // (Note: sign flipped because z_pred = -R^T * g_W)
    // where I_{3×2} = [1 0; 0 1; 0 0] for accel bias (x, y only)

    Matrix H = Matrix::zeros(3, ERROR_STATE_DIM);

    // δh/δθ = +R^T * [g_W]× (positive because of the negative in z_pred)
    Matrix skew_g = utils::skewSymmetric(gravity_);
    Matrix R_T_skew_g = R.transpose() * skew_g;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            H(i, ERR_IDX_ATT + j) = R_T_skew_g(i, j);  // Positive sign
        }
    }

    // δh/δb_a = I_{3×2}
    H(0, ERR_IDX_ACC_BIAS + 0) = 1.0;  // b_ax
    H(1, ERR_IDX_ACC_BIAS + 1) = 1.0;  // b_ay
    // H(2, :) = 0 for z-axis (z-bias not estimated)

    // Observation noise covariance
    double sigma_acc = params_.accelerometer_attitude.noise_std;
    Matrix R_noise = Matrix::identity(3) * (sigma_acc * sigma_acc);

    // Perform Kalman update
    kalmanUpdate(z_meas, z_pred, H, R_noise);

    return true;
}

// ==================== Helper Methods (Phase 4) ====================

void ESKFEstimator::Impl::propagateNominalState(const ImuData& imu, double dt) {
    // Bias correction
    Vector3 b_a_3d(accel_bias_.x(), accel_bias_.y(), 0.0);
    Vector3 a_corrected = imu.acceleration - b_a_3d;
    Vector3 w_corrected = imu.angular_velocity - gyro_bias_;

    // Get rotation matrix from current quaternion
    Matrix R = utils::quaternionToRotationMatrix(quaternion_);

    // ========== Nominal State Propagation (Euler Integration) ==========

    // Position: p[k+1] = p[k] + v[k] * dt
    Vector3 p_new = position_ + velocity_ * dt;

    // Velocity: v[k+1] = v[k] + (R * a_corrected + g_W) * dt
    // R * a_corrected transforms body-frame acceleration to world frame
    // Convert Vector3 to column matrix for matrix multiplication
    Matrix a_mat(3, 1);
    a_mat(0, 0) = a_corrected.x();
    a_mat(1, 0) = a_corrected.y();
    a_mat(2, 0) = a_corrected.z();
    Matrix a_world_mat = R * a_mat;
    Vector3 a_world(a_world_mat(0, 0), a_world_mat(1, 0), a_world_mat(2, 0));
    Vector3 v_new = velocity_ + (a_world + gravity_) * dt;

    // Quaternion: q[k+1] = q[k] + 0.5 * Omega(w_corrected) * q[k] * dt
    // Using quaternion multiplication: q_dot = 0.5 * q * [0, w]
    // Simplified: q[k+1] = q[k] * delta_q where delta_q ≈ [1, w*dt/2]
    Quaternion omega_quat(0.0, w_corrected.x(), w_corrected.y(), w_corrected.z());
    Quaternion q_dot = quaternion_ * omega_quat * 0.5;
    Quaternion q_new(
        quaternion_.w() + q_dot.w() * dt,
        quaternion_.x() + q_dot.x() * dt,
        quaternion_.y() + q_dot.y() * dt,
        quaternion_.z() + q_dot.z() * dt
    );
    q_new.normalize();  // CRITICAL: Must normalize after integration!

    // Biases: constant (random walk model, updated via noise)
    // b_g[k+1] = b_g[k]
    // b_a[k+1] = b_a[k]

    // ========== Update Nominal State ==========
    position_ = p_new;
    velocity_ = v_new;
    quaternion_ = q_new;
    // gyro_bias_ unchanged
    // accel_bias_ unchanged
}

Matrix ESKFEstimator::Impl::computeErrorStateJacobian(
    const Vector3& a_corrected,
    const Vector3& w_corrected,
    double dt) const
{
    // ========== Continuous-Time Error State Jacobian F_c (15×15) ==========
    //
    // F_c = [  0    I    0       0       0     ]  <- δp (3)
    //       [  0    0  -R[a]×    0    -R[:,:2] ]  <- δv (3)
    //       [  0    0  -[ω]×   -I       0     ]  <- δθ (3)
    //       [  0    0    0       0       0     ]  <- δb_g (3)
    //       [  0    0    0       0       0     ]  <- δb_a (2)
    //
    // State indices:
    //   0-2:  position error (δp)
    //   3-5:  velocity error (δv)
    //   6-8:  attitude error (δθ)
    //   9-11: gyro bias error (δb_g)
    //   12-13: accel bias error (δb_a)

    Matrix F_c = Matrix::zeros(ERROR_STATE_DIM, ERROR_STATE_DIM);

    // Get rotation matrix from current quaternion
    Matrix R = utils::quaternionToRotationMatrix(quaternion_);

    // ========== dp/dv = I (position depends on velocity) ==========
    // Block (0, 3) = I_3×3
    for (int i = 0; i < 3; ++i) {
        F_c(ERR_IDX_POS + i, ERR_IDX_VEL + i) = 1.0;
    }

    // ========== dv/dθ = -R * [a_corrected]× ==========
    // Velocity error depends on attitude error through rotation of acceleration
    Matrix skew_a = utils::skewSymmetric(a_corrected);
    Matrix R_skew_a = R * skew_a;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            F_c(ERR_IDX_VEL + i, ERR_IDX_ATT + j) = -R_skew_a(i, j);
        }
    }

    // ========== dv/db_a = -R[:, 0:2] ==========
    // Velocity error depends on accel bias (first 2 columns of R)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 2; ++j) {
            F_c(ERR_IDX_VEL + i, ERR_IDX_ACC_BIAS + j) = -R(i, j);
        }
    }

    // ========== dθ/dθ = -[w_corrected]× ==========
    // Attitude error kinematics
    Matrix skew_w = utils::skewSymmetric(w_corrected);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            F_c(ERR_IDX_ATT + i, ERR_IDX_ATT + j) = -skew_w(i, j);
        }
    }

    // ========== dθ/db_g = -I ==========
    // Attitude error depends on gyro bias
    for (int i = 0; i < 3; ++i) {
        F_c(ERR_IDX_ATT + i, ERR_IDX_GYRO_BIAS + i) = -1.0;
    }

    // Biases: db_g/db_g = 0, db_a/db_a = 0 (random walk, already zero)

    // ========== Discretization: F_d = I + F_c * dt ==========
    Matrix F_d = Matrix::identity(ERROR_STATE_DIM) + F_c * dt;

    return F_d;
}

Matrix ESKFEstimator::Impl::computeProcessNoise(double dt) const {
    // ========== Discrete-Time Process Noise Covariance Q_d (15×15) ==========
    //
    // Q_d = diag([0, 0, 0,                        <- position (no direct noise)
    //             σ_a², σ_a², σ_a²,               <- velocity (accel noise)
    //             σ_ω², σ_ω², σ_ω²,               <- attitude (gyro noise)
    //             σ_bg², σ_bg², σ_bg²,            <- gyro bias drift
    //             σ_ba², σ_ba²]) * dt             <- accel bias drift

    Matrix Q_d = Matrix::zeros(ERROR_STATE_DIM, ERROR_STATE_DIM);

    // Get noise parameters
    double sigma_a = params_.process_noise.sigma_a;      // Accelerometer noise
    double sigma_w = params_.process_noise.sigma_omega;  // Gyro noise
    double sigma_bg = params_.process_noise.sigma_bg;    // Gyro bias random walk
    double sigma_ba = params_.process_noise.sigma_ba;    // Accel bias random walk

    // Position: no direct process noise (driven by velocity)
    // Q_d(0:2, 0:2) = 0

    // Velocity noise (from accelerometer noise)
    double var_a = sigma_a * sigma_a * dt;
    for (int i = 0; i < 3; ++i) {
        Q_d(ERR_IDX_VEL + i, ERR_IDX_VEL + i) = var_a;
    }

    // Attitude noise (from gyro noise)
    double var_w = sigma_w * sigma_w * dt;
    for (int i = 0; i < 3; ++i) {
        Q_d(ERR_IDX_ATT + i, ERR_IDX_ATT + i) = var_w;
    }

    // Gyro bias random walk
    double var_bg = sigma_bg * sigma_bg * dt;
    for (int i = 0; i < 3; ++i) {
        Q_d(ERR_IDX_GYRO_BIAS + i, ERR_IDX_GYRO_BIAS + i) = var_bg;
    }

    // Accel bias random walk (only x, y)
    double var_ba = sigma_ba * sigma_ba * dt;
    for (int i = 0; i < 2; ++i) {
        Q_d(ERR_IDX_ACC_BIAS + i, ERR_IDX_ACC_BIAS + i) = var_ba;
    }

    return Q_d;
}

void ESKFEstimator::Impl::kalmanUpdate(
    const Matrix& z_meas,
    const Matrix& z_pred,
    const Matrix& H,
    const Matrix& R)
{
    // ========== Kalman Filter Update (Phase 5) ==========
    //
    // Standard Kalman filter update equations:
    // 1. Innovation: y = z_meas - z_pred
    // 2. Innovation covariance: S = H * P * H^T + R
    // 3. Kalman gain: K = P * H^T * S^(-1)
    // 4. Error state update: δx = K * y
    // 5. Covariance update (Joseph form): P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    // 6. Inject error state into nominal state
    // 7. Reset error state to zero (implicit in ESKF)

    // Step 1: Compute innovation (measurement residual)
    Matrix y = z_meas - z_pred;

    // Step 2: Compute innovation covariance
    // S = H * P * H^T + R
    Matrix S = H * P_ * H.transpose() + R;

    // Step 3: Outlier rejection using Mahalanobis distance
    if (!checkMahalanobisDistance(y, S)) {
        rejected_count_++;
        return;  // Reject this measurement
    }

    // Step 4: Compute Kalman gain
    // K = P * H^T * S^(-1)
    Matrix S_inv = S.inverse();
    Matrix K = P_ * H.transpose() * S_inv;

    // Step 5: Compute error state update
    Matrix dx_error = K * y;

    // Step 6: Update error state covariance using Joseph form (numerically stable)
    // P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    Matrix I_KH = Matrix::identity(ERROR_STATE_DIM) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();

    // Enforce symmetry and positive definiteness
    P_ = utils::enforceCovarianceProperties(P_, 1e-9);

    // Step 7: Inject error state into nominal state
    injectErrorState(dx_error);

    // Error state is implicitly reset to zero (ESKF principle)
}

void ESKFEstimator::Impl::injectErrorState(const Matrix& dx_error) {
    // ========== Error State Injection (Phase 5) ==========
    //
    // ESKF key operation: inject error state into nominal state
    //
    // For position, velocity, and biases: simple addition
    // For attitude: quaternion multiplication (CRITICAL!)
    //
    // δx = [δp, δv, δθ, δb_g, δb_a]^T

    // ========== Position: p = p + δp ==========
    Vector3 delta_p(dx_error(ERR_IDX_POS, 0),
                    dx_error(ERR_IDX_POS + 1, 0),
                    dx_error(ERR_IDX_POS + 2, 0));
    position_ = position_ + delta_p;

    // ========== Velocity: v = v + δv ==========
    Vector3 delta_v(dx_error(ERR_IDX_VEL, 0),
                    dx_error(ERR_IDX_VEL + 1, 0),
                    dx_error(ERR_IDX_VEL + 2, 0));
    velocity_ = velocity_ + delta_v;

    // ========== Attitude: q = q ⊗ δq (quaternion multiplication!) ==========
    // Convert small angle vector to quaternion
    Vector3 delta_theta(dx_error(ERR_IDX_ATT, 0),
                        dx_error(ERR_IDX_ATT + 1, 0),
                        dx_error(ERR_IDX_ATT + 2, 0));
    Quaternion delta_q = utils::smallAngleQuaternion(delta_theta);

    // Quaternion multiplication: q_new = q ⊗ δq
    // Order matters! This is q * delta_q (local perturbation)
    quaternion_ = quaternion_ * delta_q;
    quaternion_.normalize();  // CRITICAL: Always normalize after quaternion operations!

    // ========== Gyro bias: b_g = b_g + δb_g ==========
    Vector3 delta_bg(dx_error(ERR_IDX_GYRO_BIAS, 0),
                     dx_error(ERR_IDX_GYRO_BIAS + 1, 0),
                     dx_error(ERR_IDX_GYRO_BIAS + 2, 0));
    gyro_bias_ = gyro_bias_ + delta_bg;

    // ========== Accel bias: b_a = b_a + δb_a (only x, y) ==========
    Vector3 delta_ba(dx_error(ERR_IDX_ACC_BIAS, 0),
                     dx_error(ERR_IDX_ACC_BIAS + 1, 0),
                     0.0);  // z component is not estimated
    accel_bias_ = accel_bias_ + delta_ba;
}

bool ESKFEstimator::Impl::checkMahalanobisDistance(
    const Matrix& innovation,
    const Matrix& S) const
{
    // ========== Mahalanobis Distance Check (Phase 5) ==========
    //
    // Mahalanobis distance: d² = y^T * S^(-1) * y
    //
    // Used for outlier rejection:
    // - If d² > threshold, the measurement is considered an outlier
    // - The threshold is based on chi-squared distribution
    //
    // Typical thresholds (95% confidence):
    // - 1 DOF: 3.84
    // - 2 DOF: 5.99
    // - 3 DOF: 7.81

    if (!params_.outlier_rejection.enabled) {
        return true;  // Accept all measurements
    }

    // Compute Mahalanobis distance squared
    double d_sq = utils::mahalanobisDistance(innovation, S);

    // Check against threshold
    double threshold = params_.outlier_rejection.mahalanobis_threshold;

    if (d_sq > threshold) {
        // Outlier detected
        // Debug: uncomment to see Mahalanobis rejections
        // std::cerr << "Mahalanobis rejected: d²=" << d_sq << " > " << threshold << "\n";
        return false;
    }

    return true;  // Measurement accepted
}

// ==================== Public API Implementation ====================

ESKFEstimator::ESKFEstimator(const ESKFParameters& params)
    : pimpl_(std::make_unique<Impl>(params))
{
}

ESKFEstimator::~ESKFEstimator() = default;

bool ESKFEstimator::initialize(const EstimatorState& initial_state) {
    return pimpl_->initialize(initial_state);
}

bool ESKFEstimator::initializeDefault() {
    return pimpl_->initializeDefault();
}

void ESKFEstimator::reset() {
    pimpl_->reset();
}

bool ESKFEstimator::isInitialized() const {
    return pimpl_->isInitialized();
}

EstimatorStatus ESKFEstimator::getStatus() const {
    return pimpl_->getStatus();
}

void ESKFEstimator::processImu(const ImuData& imu) {
    pimpl_->processImu(imu);
}

bool ESKFEstimator::processMagnetometer(const MagnetometerData& mag) {
    return pimpl_->processMagnetometer(mag);
}

bool ESKFEstimator::processBarometer(const BarometerData& baro) {
    return pimpl_->processBarometer(baro);
}

bool ESKFEstimator::processTof(const TofData& tof) {
    return pimpl_->processTof(tof);
}

bool ESKFEstimator::processOpticalFlow(const OpticalFlowData& flow) {
    return pimpl_->processOpticalFlow(flow);
}

bool ESKFEstimator::processAccelerometer(const Vector3& acceleration) {
    return pimpl_->processAccelerometer(acceleration);
}

EstimatorState ESKFEstimator::getState() const {
    return pimpl_->getState();
}

Vector3 ESKFEstimator::getPosition() const {
    return pimpl_->getPosition();
}

Vector3 ESKFEstimator::getVelocity() const {
    return pimpl_->getVelocity();
}

Quaternion ESKFEstimator::getQuaternion() const {
    return pimpl_->getQuaternion();
}

Vector3 ESKFEstimator::getEulerAngles() const {
    return pimpl_->getEulerAngles();
}

Vector3 ESKFEstimator::getGyroBias() const {
    return pimpl_->getGyroBias();
}

Vector3 ESKFEstimator::getAccelBias() const {
    return pimpl_->getAccelBias();
}

Matrix ESKFEstimator::getCovariance() const {
    return pimpl_->getCovariance();
}

void ESKFEstimator::connectSensorSuite(std::shared_ptr<SensorSuite> sensors) {
    // TODO: Implement sensor suite integration
    std::cerr << "WARNING: connectSensorSuite() not yet implemented\n";
}

void ESKFEstimator::disconnectSensorSuite() {
    // TODO: Implement sensor suite integration
    std::cerr << "WARNING: disconnectSensorSuite() not yet implemented\n";
}

void ESKFEstimator::updateParameters(const ESKFParameters& params) {
    pimpl_->updateParameters(params);
}

const ESKFParameters& ESKFEstimator::getParameters() const {
    return pimpl_->getParameters();
}

double ESKFEstimator::getTimeSinceLastImu() const {
    return pimpl_->getTimeSinceLastImu();
}

uint32_t ESKFEstimator::getRejectedMeasurementCount() const {
    return pimpl_->getRejectedMeasurementCount();
}

void ESKFEstimator::resetDiagnostics() {
    pimpl_->resetDiagnostics();
}

} // namespace estimator
} // namespace stampfly
