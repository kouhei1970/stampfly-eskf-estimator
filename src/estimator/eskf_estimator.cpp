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

    // Sensor processing (to be implemented in future phases)
    void processImu(const ImuData& imu);
    bool processMagnetometer(const MagnetometerData& mag);
    bool processBarometer(const BarometerData& baro);
    bool processTof(const TofData& tof);
    bool processOpticalFlow(const OpticalFlowData& flow);

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
      rejected_count_(0)
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

// Sensor processing stubs (to be implemented in future phases)

void ESKFEstimator::Impl::processImu(const ImuData& imu) {
    if (!initialized_) {
        std::cerr << "ERROR: processImu() called before initialization\n";
        return;
    }

    // TODO: Implement in Phase 4
    std::cerr << "WARNING: processImu() not yet implemented\n";
}

bool ESKFEstimator::Impl::processMagnetometer(const MagnetometerData& mag) {
    if (!initialized_) {
        std::cerr << "ERROR: processMagnetometer() called before initialization\n";
        return false;
    }

    // TODO: Implement in Phase 6
    std::cerr << "WARNING: processMagnetometer() not yet implemented\n";
    return false;
}

bool ESKFEstimator::Impl::processBarometer(const BarometerData& baro) {
    if (!initialized_) {
        std::cerr << "ERROR: processBarometer() called before initialization\n";
        return false;
    }

    // TODO: Implement in Phase 7
    std::cerr << "WARNING: processBarometer() not yet implemented\n";
    return false;
}

bool ESKFEstimator::Impl::processTof(const TofData& tof) {
    if (!initialized_) {
        std::cerr << "ERROR: processTof() called before initialization\n";
        return false;
    }

    // TODO: Implement in Phase 7
    std::cerr << "WARNING: processTof() not yet implemented\n";
    return false;
}

bool ESKFEstimator::Impl::processOpticalFlow(const OpticalFlowData& flow) {
    if (!initialized_) {
        std::cerr << "ERROR: processOpticalFlow() called before initialization\n";
        return false;
    }

    // TODO: Implement in Phase 8
    std::cerr << "WARNING: processOpticalFlow() not yet implemented\n";
    return false;
}

// Helper method stubs (to be implemented in future phases)

void ESKFEstimator::Impl::propagateNominalState(const ImuData& imu, double dt) {
    // TODO: Implement in Phase 4
}

Matrix ESKFEstimator::Impl::computeErrorStateJacobian(
    const Vector3& a_corrected,
    const Vector3& w_corrected,
    double dt) const
{
    // TODO: Implement in Phase 4
    return Matrix::identity(ERROR_STATE_DIM);
}

Matrix ESKFEstimator::Impl::computeProcessNoise(double dt) const {
    // TODO: Implement in Phase 4
    return Matrix::identity(ERROR_STATE_DIM);
}

void ESKFEstimator::Impl::kalmanUpdate(
    const Matrix& z_meas,
    const Matrix& z_pred,
    const Matrix& H,
    const Matrix& R)
{
    // TODO: Implement in Phase 5
}

void ESKFEstimator::Impl::injectErrorState(const Matrix& dx_error) {
    // TODO: Implement in Phase 5
}

bool ESKFEstimator::Impl::checkMahalanobisDistance(
    const Matrix& innovation,
    const Matrix& S) const
{
    // TODO: Implement in Phase 5
    return true;
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
