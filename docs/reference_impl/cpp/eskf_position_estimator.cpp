/**
 * @file eskf_position_estimator.cpp
 * @brief クアッドコプター位置・姿勢推定 ESKF実装（C++版）
 */

#include "eskf_position_estimator.hpp"
#include <iostream>

namespace quadcopter {

ESKFPositionEstimator::ESKFPositionEstimator(const ESKFParameters& params)
    : params_(params),
      x_nominal_(VectorXd::Zero(STATE_DIM)),
      dx_error_(VectorXd::Zero(ERROR_STATE_DIM)),
      P_(MatrixXd::Identity(ERROR_STATE_DIM, ERROR_STATE_DIM))
{
    // ノミナル状態の初期化
    x_nominal_[6] = 1.0;  // q_w = 1（単位クォータニオン）

    // エラー状態共分散行列の初期化
    P_ = initializeCovariance();

    // 地磁気ベクトル（ワールド座標系）
    m_W_ << params_.m_N, 0.0, params_.m_D;

    // 重力ベクトル（ワールド座標系、NED座標系）
    g_W_ << 0.0, 0.0, params_.g;
}

ESKFPositionEstimator::MatrixXd ESKFPositionEstimator::initializeCovariance() const
{
    MatrixXd P = MatrixXd::Identity(ERROR_STATE_DIM, ERROR_STATE_DIM);

    // 位置
    P.block<3, 3>(0, 0) *= params_.sigma_p_init * params_.sigma_p_init;
    // 速度
    P.block<3, 3>(3, 3) *= params_.sigma_v_init * params_.sigma_v_init;
    // 姿勢（回転ベクトル、3次元）
    P.block<3, 3>(6, 6) *= params_.sigma_theta_init * params_.sigma_theta_init;
    // ジャイロバイアス
    P.block<3, 3>(9, 9) *= params_.sigma_bg_init * params_.sigma_bg_init;
    // 加速度バイアス
    P.block<2, 2>(12, 12) *= params_.sigma_ba_init * params_.sigma_ba_init;

    return P;
}

// ========== ユーティリティ関数 ==========

ESKFPositionEstimator::Matrix3d ESKFPositionEstimator::quaternionToRotationMatrix(const Vector4d& q)
{
    double q_w = q(0), q_x = q(1), q_y = q(2), q_z = q(3);

    Matrix3d R;
    R << 1 - 2*(q_y*q_y + q_z*q_z),     2*(q_x*q_y - q_w*q_z),     2*(q_x*q_z + q_w*q_y),
             2*(q_x*q_y + q_w*q_z), 1 - 2*(q_x*q_x + q_z*q_z),     2*(q_y*q_z - q_w*q_x),
             2*(q_x*q_z - q_w*q_y),     2*(q_y*q_z + q_w*q_x), 1 - 2*(q_x*q_x + q_y*q_y);

    return R;
}

ESKFPositionEstimator::Vector4d ESKFPositionEstimator::normalizeQuaternion(const Vector4d& q)
{
    double norm = q.norm();
    if (norm < 1e-9) {
        return Vector4d(1.0, 0.0, 0.0, 0.0);
    }
    return q / norm;
}

ESKFPositionEstimator::Matrix4d ESKFPositionEstimator::omegaMatrix(const Vector3d& w)
{
    Matrix4d Omega;
    Omega <<     0, -w(0), -w(1), -w(2),
              w(0),     0,  w(2), -w(1),
              w(1), -w(2),     0,  w(0),
              w(2),  w(1), -w(0),     0;
    return Omega;
}

ESKFPositionEstimator::Matrix3d ESKFPositionEstimator::skewSymmetric(const Vector3d& v)
{
    Matrix3d S;
    S <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return S;
}

ESKFPositionEstimator::Vector4d ESKFPositionEstimator::smallAngleQuaternion(const Vector3d& delta_theta)
{
    double theta_norm = delta_theta.norm();

    if (theta_norm < 1e-6) {
        // 小角度近似
        Vector4d dq;
        dq << 1.0, 0.5*delta_theta(0), 0.5*delta_theta(1), 0.5*delta_theta(2);
        return normalizeQuaternion(dq);
    } else {
        // 完全な変換
        double half_theta = theta_norm / 2.0;
        Vector4d dq;
        dq << std::cos(half_theta),
              (delta_theta(0) / theta_norm) * std::sin(half_theta),
              (delta_theta(1) / theta_norm) * std::sin(half_theta),
              (delta_theta(2) / theta_norm) * std::sin(half_theta);
        return dq;
    }
}

ESKFPositionEstimator::MatrixXd ESKFPositionEstimator::makeSymmetric(const MatrixXd& P)
{
    return (P + P.transpose()) / 2.0;
}

// ========== 予測ステップ ==========

void ESKFPositionEstimator::predict(const Vector3d& a_meas, const Vector3d& w_meas, double dt)
{
    // ステップ1: ノミナル状態の伝播
    propagateNominalState(a_meas, w_meas, dt);

    // ステップ2: エラー状態共分散の伝播
    Vector4d q = x_nominal_.segment<4>(6);
    Vector3d b_g = x_nominal_.segment<3>(10);
    Eigen::Vector2d b_a = x_nominal_.segment<2>(13);

    Vector3d b_a_3d(b_a(0), b_a(1), 0.0);
    Vector3d a_corrected = a_meas - b_a_3d;
    Vector3d w_corrected = w_meas - b_g;

    // 連続時間システム行列 F_c
    MatrixXd F_c = computeContinuousErrorStateJacobian(q, a_corrected, w_corrected);

    // 離散化: F_d = I + F_c * dt
    MatrixXd F_d = MatrixXd::Identity(ERROR_STATE_DIM, ERROR_STATE_DIM) + F_c * dt;

    // プロセスノイズ共分散 Q_d
    MatrixXd Q_d = computeProcessNoiseCov(dt);

    // 共分散伝播
    P_ = F_d * P_ * F_d.transpose() + Q_d;
    P_ = makeSymmetric(P_);

    // エラー状態は0のまま（リセット後）
}

void ESKFPositionEstimator::propagateNominalState(const Vector3d& a_meas, const Vector3d& w_meas, double dt)
{
    // 現在のノミナル状態を取り出す
    Vector3d p = x_nominal_.segment<3>(0);
    Vector3d v = x_nominal_.segment<3>(3);
    Vector4d q = x_nominal_.segment<4>(6);
    Vector3d b_g = x_nominal_.segment<3>(10);
    Eigen::Vector2d b_a = x_nominal_.segment<2>(13);

    // バイアス補正
    Vector3d b_a_3d(b_a(0), b_a(1), 0.0);
    Vector3d a_corrected = a_meas - b_a_3d;
    Vector3d w_corrected = w_meas - b_g;

    // 回転行列
    Matrix3d R = quaternionToRotationMatrix(q);

    // ノミナル状態の更新
    Vector3d p_new = p + v * dt;
    Vector3d v_new = v + (R * a_corrected + g_W_) * dt;
    Vector4d q_new = q + 0.5 * omegaMatrix(w_corrected) * q * dt;
    q_new = normalizeQuaternion(q_new);

    // ノミナル状態を更新
    x_nominal_.segment<3>(0) = p_new;
    x_nominal_.segment<3>(3) = v_new;
    x_nominal_.segment<4>(6) = q_new;
    // バイアスは変わらず
}

ESKFPositionEstimator::MatrixXd ESKFPositionEstimator::computeContinuousErrorStateJacobian(
    const Vector4d& q, const Vector3d& a_corrected, const Vector3d& w_corrected) const
{
    MatrixXd F_c = MatrixXd::Zero(ERROR_STATE_DIM, ERROR_STATE_DIM);

    Matrix3d R = quaternionToRotationMatrix(q);

    // dp/dv = I
    F_c.block<3, 3>(0, 3) = Matrix3d::Identity();

    // dv/dθ = -R * [a_corrected]_×
    F_c.block<3, 3>(3, 6) = -R * skewSymmetric(a_corrected);

    // dv/db_a = -R[:, 0:2]
    F_c.block<3, 2>(3, 12) = -R.block<3, 2>(0, 0);

    // dθ/dθ = -[w_corrected]_×
    F_c.block<3, 3>(6, 6) = -skewSymmetric(w_corrected);

    // dθ/db_g = -I
    F_c.block<3, 3>(6, 9) = -Matrix3d::Identity();

    // バイアスは定数
    // db_g/db_g = 0
    // db_a/db_a = 0

    return F_c;
}

ESKFPositionEstimator::MatrixXd ESKFPositionEstimator::computeProcessNoiseCov(double dt) const
{
    MatrixXd Q = MatrixXd::Zero(ERROR_STATE_DIM, ERROR_STATE_DIM);

    // 位置: ノイズなし（速度積分による）
    // Q.block<3, 3>(0, 0) = 0

    // 速度
    Q.block<3, 3>(3, 3) = Matrix3d::Identity() * params_.sigma_a_n * params_.sigma_a_n * dt;

    // 姿勢（回転ベクトル）
    Q.block<3, 3>(6, 6) = Matrix3d::Identity() * params_.sigma_omega_n * params_.sigma_omega_n * dt;

    // ジャイロバイアス
    Q.block<3, 3>(9, 9) = Matrix3d::Identity() * params_.sigma_bg_n * params_.sigma_bg_n * dt;

    // 加速度バイアス
    Q.block<2, 2>(12, 12) = Eigen::Matrix2d::Identity() * params_.sigma_ba_n * params_.sigma_ba_n * dt;

    return Q;
}

// ========== 更新ステップ ==========

void ESKFPositionEstimator::updateAccelerometer(const Vector3d& z_acc)
{
    Vector4d q = x_nominal_.segment<4>(6);
    Eigen::Vector2d b_a = x_nominal_.segment<2>(13);
    Matrix3d R = quaternionToRotationMatrix(q);

    // 観測予測
    Vector3d b_a_3d(b_a(0), b_a(1), 0.0);
    Vector3d z_pred = R.transpose() * g_W_ + b_a_3d;

    // 観測ヤコビアン H
    Eigen::Matrix<double, 3, ERROR_STATE_DIM> H = computeAccelerometerJacobian(q);

    // 観測ノイズ共分散
    Matrix3d R_noise = Matrix3d::Identity() * params_.sigma_acc * params_.sigma_acc;

    // カルマンゲインと更新、エラー状態リセット
    kalmanUpdateAndReset(z_acc, z_pred, H, R_noise);
}

Eigen::Matrix<double, 3, ESKFPositionEstimator::ERROR_STATE_DIM>
ESKFPositionEstimator::computeAccelerometerJacobian(const Vector4d& q) const
{
    Eigen::Matrix<double, 3, ERROR_STATE_DIM> H = MatrixXd::Zero(3, ERROR_STATE_DIM);

    Matrix3d R = quaternionToRotationMatrix(q);

    // h_acc = R^T * g_W + [b_ax, b_ay, 0]^T
    // δh/δθ = -R^T * [g_W]_×
    H.block<3, 3>(0, 6) = -R.transpose() * skewSymmetric(g_W_);

    // δh/δb_a
    H(0, 12) = 1.0;  // b_ax
    H(1, 13) = 1.0;  // b_ay
    // H(2, :) = 0（z軸バイアスは推定しない）

    return H;
}

void ESKFPositionEstimator::updateMagnetometer(const Vector3d& z_mag)
{
    Vector4d q = x_nominal_.segment<4>(6);
    Matrix3d R = quaternionToRotationMatrix(q);

    // 観測予測
    Vector3d z_pred = R.transpose() * m_W_;

    // 観測ヤコビアン H
    Eigen::Matrix<double, 3, ERROR_STATE_DIM> H = computeMagnetometerJacobian(q);

    // 観測ノイズ共分散
    Matrix3d R_noise = Matrix3d::Identity() * params_.sigma_mag * params_.sigma_mag;

    // カルマンゲインと更新、エラー状態リセット
    kalmanUpdateAndReset(z_mag, z_pred, H, R_noise);
}

Eigen::Matrix<double, 3, ESKFPositionEstimator::ERROR_STATE_DIM>
ESKFPositionEstimator::computeMagnetometerJacobian(const Vector4d& q) const
{
    Eigen::Matrix<double, 3, ERROR_STATE_DIM> H = MatrixXd::Zero(3, ERROR_STATE_DIM);

    Matrix3d R = quaternionToRotationMatrix(q);

    // h_mag = R^T * m_W
    // δh/δθ = -R^T * [m_W]_×
    H.block<3, 3>(0, 6) = -R.transpose() * skewSymmetric(m_W_);

    return H;
}

void ESKFPositionEstimator::updateBarometer(double z_baro)
{
    double p_z = x_nominal_(2);

    // 観測予測
    double z_pred = p_z;

    // 観測ヤコビアン H (1x15)
    Eigen::Matrix<double, 1, ERROR_STATE_DIM> H = MatrixXd::Zero(1, ERROR_STATE_DIM);
    H(0, 2) = 1.0;  // δp_z

    // 観測ノイズ共分散
    Eigen::Matrix<double, 1, 1> R_noise;
    R_noise << params_.sigma_baro * params_.sigma_baro;

    // カルマンゲインと更新、エラー状態リセット
    Eigen::Matrix<double, 1, 1> z_meas_vec, z_pred_vec;
    z_meas_vec << z_baro;
    z_pred_vec << z_pred;
    kalmanUpdateAndReset(z_meas_vec, z_pred_vec, H, R_noise);
}

void ESKFPositionEstimator::updateToF(double z_tof, bool use_tilt_compensation)
{
    double p_z = x_nominal_(2);
    Vector4d q = x_nominal_.segment<4>(6);

    Eigen::Matrix<double, 1, ERROR_STATE_DIM> H = MatrixXd::Zero(1, ERROR_STATE_DIM);
    double z_pred;

    if (use_tilt_compensation) {
        Matrix3d R = quaternionToRotationMatrix(q);
        double R_22 = R(2, 2);

        if (std::abs(R_22) < 0.1) {
            // 大きく傾いている場合はスキップ
            return;
        }

        z_pred = p_z / R_22;

        // ヤコビアン（簡易実装）
        H(0, 2) = 1.0 / R_22;  // δp_z
        // δθに関する微分は省略（完全実装は複雑）
    } else {
        // 簡易版（小傾斜近似）
        z_pred = p_z;
        H(0, 2) = 1.0;
    }

    // 観測ノイズ共分散
    Eigen::Matrix<double, 1, 1> R_noise;
    R_noise << params_.sigma_tof * params_.sigma_tof;

    // カルマンゲインと更新、エラー状態リセット
    Eigen::Matrix<double, 1, 1> z_meas_vec, z_pred_vec;
    z_meas_vec << z_tof;
    z_pred_vec << z_pred;
    kalmanUpdateAndReset(z_meas_vec, z_pred_vec, H, R_noise);
}

void ESKFPositionEstimator::updateOpticalFlow(const Eigen::Vector2d& z_flow, const Vector3d& w_meas)
{
    double p_z = x_nominal_(2);
    Vector3d v = x_nominal_.segment<3>(3);
    Vector4d q = x_nominal_.segment<4>(6);
    Vector3d b_g = x_nominal_.segment<3>(10);

    // 高度が低すぎる場合はスキップ
    if (p_z < 0.1) {
        return;
    }

    // バイアス補正済み角速度
    Vector3d w_corrected = w_meas - b_g;

    // ボディ座標系での速度
    Matrix3d R = quaternionToRotationMatrix(q);
    Vector3d v_B = R.transpose() * v;

    // 観測予測
    Eigen::Vector2d z_pred;
    z_pred(0) = (v_B(0) - w_corrected(1) * p_z) / p_z;
    z_pred(1) = (v_B(1) + w_corrected(0) * p_z) / p_z;

    // 観測ヤコビアン H (2x15)
    Eigen::Matrix<double, 2, ERROR_STATE_DIM> H = computeOpticalFlowJacobian(q, v, p_z, w_corrected);

    // 観測ノイズ共分散
    Eigen::Matrix2d R_noise = Eigen::Matrix2d::Identity() * params_.sigma_flow * params_.sigma_flow;

    // カルマンゲインと更新、エラー状態リセット
    kalmanUpdateAndReset(z_flow, z_pred, H, R_noise);
}

Eigen::Matrix<double, 2, ESKFPositionEstimator::ERROR_STATE_DIM>
ESKFPositionEstimator::computeOpticalFlowJacobian(
    const Vector4d& q, const Vector3d& v, double p_z, const Vector3d& w_corrected) const
{
    Eigen::Matrix<double, 2, ERROR_STATE_DIM> H = MatrixXd::Zero(2, ERROR_STATE_DIM);

    Matrix3d R = quaternionToRotationMatrix(q);
    Vector3d v_B = R.transpose() * v;

    // δh/δp_z
    H(0, 2) = -(v_B(0) - w_corrected(1)*p_z) / (p_z * p_z);
    H(1, 2) = -(v_B(1) + w_corrected(0)*p_z) / (p_z * p_z);

    // δh/δv = (1/p_z) * δv_B/δv = (1/p_z) * R^T
    H.block<2, 3>(0, 3) = (1.0 / p_z) * R.transpose().block<2, 3>(0, 0);

    // δh/δθ: v_Bのδθに関する微分
    // δv_B = -[v_B]_× * δθ
    Matrix3d dv_B_dtheta = -skewSymmetric(v_B);
    H.block<2, 3>(0, 6) = (1.0 / p_z) * dv_B_dtheta.block<2, 3>(0, 0);

    // δh/δb_g: 角速度バイアスの影響
    H(0, 10) = -1.0;  // b_gy
    H(1, 9) = 1.0;    // b_gx

    return H;
}

void ESKFPositionEstimator::kalmanUpdateAndReset(const VectorXd& z_meas, const VectorXd& z_pred,
                                                  const MatrixXd& H, const MatrixXd& R)
{
    // イノベーション（残差）
    VectorXd y = z_meas - z_pred;

    // イノベーション共分散
    MatrixXd S = H * P_ * H.transpose() + R;

    // 外れ値検出（マハラノビス距離）
    double mahalanobis_dist_sq = (y.transpose() * S.inverse() * y)(0, 0);

    if (mahalanobis_dist_sq > params_.mahalanobis_threshold) {
        std::cout << "外れ値検出: マハラノビス距離^2 = " << mahalanobis_dist_sq << std::endl;
        return;
    }

    // カルマンゲイン
    MatrixXd K = P_ * H.transpose() * S.inverse();

    // エラー状態の更新
    dx_error_ = K * y;

    // エラー共分散の更新（Joseph形式）
    MatrixXd I = MatrixXd::Identity(ERROR_STATE_DIM, ERROR_STATE_DIM);
    MatrixXd I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    P_ = makeSymmetric(P_);

    // エラー状態をノミナル状態に注入
    injectErrorState(dx_error_);

    // エラー状態をリセット
    dx_error_.setZero();
}

void ESKFPositionEstimator::injectErrorState(const VectorXd& dx)
{
    // 位置
    x_nominal_.segment<3>(0) += dx.segment<3>(0);

    // 速度
    x_nominal_.segment<3>(3) += dx.segment<3>(3);

    // 姿勢（回転ベクトル → クォータニオン → 合成）
    Vector3d delta_theta = dx.segment<3>(6);
    Vector4d delta_q = smallAngleQuaternion(delta_theta);

    Vector4d q = x_nominal_.segment<4>(6);
    // クォータニオン乗算: q_new = q ⊗ delta_q
    Vector4d q_new;
    q_new(0) = q(0)*delta_q(0) - q(1)*delta_q(1) - q(2)*delta_q(2) - q(3)*delta_q(3);
    q_new(1) = q(0)*delta_q(1) + q(1)*delta_q(0) + q(2)*delta_q(3) - q(3)*delta_q(2);
    q_new(2) = q(0)*delta_q(2) - q(1)*delta_q(3) + q(2)*delta_q(0) + q(3)*delta_q(1);
    q_new(3) = q(0)*delta_q(3) + q(1)*delta_q(2) - q(2)*delta_q(1) + q(3)*delta_q(0);

    x_nominal_.segment<4>(6) = normalizeQuaternion(q_new);

    // ジャイロバイアス
    x_nominal_.segment<3>(10) += dx.segment<3>(9);

    // 加速度バイアス
    x_nominal_.segment<2>(13) += dx.segment<2>(12);
}

// ========== 状態取得 ==========

ESKFPositionEstimator::Vector3d ESKFPositionEstimator::getEulerAngles() const
{
    Vector4d q = x_nominal_.segment<4>(6);
    double q_w = q(0), q_x = q(1), q_y = q(2), q_z = q(3);

    // ロール（x軸周り）
    double roll = std::atan2(2*(q_w*q_x + q_y*q_z), 1 - 2*(q_x*q_x + q_y*q_y));

    // ピッチ（y軸周り）
    double pitch = std::asin(2*(q_w*q_y - q_z*q_x));

    // ヨー（z軸周り）
    double yaw = std::atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y*q_y + q_z*q_z));

    return Vector3d(roll, pitch, yaw);
}

ESKFPositionEstimator::Vector3d ESKFPositionEstimator::getPositionStd() const
{
    return Vector3d(std::sqrt(P_(0, 0)), std::sqrt(P_(1, 1)), std::sqrt(P_(2, 2)));
}

}  // namespace quadcopter
