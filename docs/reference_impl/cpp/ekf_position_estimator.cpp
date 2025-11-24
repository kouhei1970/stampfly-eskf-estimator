/**
 * @file ekf_position_estimator.cpp
 * @brief クアッドコプター位置・姿勢推定 EKF実装（C++版）
 */

#include "ekf_position_estimator.hpp"
#include <iostream>

namespace quadcopter {

EKFPositionEstimator::EKFPositionEstimator(const EKFParameters& params)
    : params_(params), x_(VectorXd::Zero(STATE_DIM)), P_(MatrixXd::Identity(STATE_DIM, STATE_DIM))
{
    // 状態ベクトルの初期化
    x_[6] = 1.0;  // q_w = 1（単位クォータニオン）

    // 誤差共分散行列の初期化
    P_ = initializeCovariance();

    // 地磁気ベクトル（ワールド座標系）
    m_W_ << params_.m_N, 0.0, params_.m_D;

    // 重力ベクトル（ワールド座標系、NED座標系）
    g_W_ << 0.0, 0.0, params_.g;
}

MatrixXd EKFPositionEstimator::initializeCovariance() const
{
    MatrixXd P = MatrixXd::Identity(STATE_DIM, STATE_DIM);

    // 位置
    P.block<3, 3>(0, 0) *= params_.sigma_p_init * params_.sigma_p_init;
    // 速度
    P.block<3, 3>(3, 3) *= params_.sigma_v_init * params_.sigma_v_init;
    // クォータニオン
    P.block<4, 4>(6, 6) *= params_.sigma_q_init * params_.sigma_q_init;
    // ジャイロバイアス
    P.block<3, 3>(10, 10) *= params_.sigma_bg_init * params_.sigma_bg_init;
    // 加速度バイアス
    P.block<2, 2>(13, 13) *= params_.sigma_ba_init * params_.sigma_ba_init;

    return P;
}

// ========== ユーティリティ関数 ==========

Matrix3d EKFPositionEstimator::quaternionToRotationMatrix(const Vector4d& q)
{
    double q_w = q(0), q_x = q(1), q_y = q(2), q_z = q(3);

    Matrix3d R;
    R << 1 - 2*(q_y*q_y + q_z*q_z),     2*(q_x*q_y - q_w*q_z),     2*(q_x*q_z + q_w*q_y),
             2*(q_x*q_y + q_w*q_z), 1 - 2*(q_x*q_x + q_z*q_z),     2*(q_y*q_z - q_w*q_x),
             2*(q_x*q_z - q_w*q_y),     2*(q_y*q_z + q_w*q_x), 1 - 2*(q_x*q_x + q_y*q_y);

    return R;
}

EKFPositionEstimator::Vector4d EKFPositionEstimator::normalizeQuaternion(const Vector4d& q)
{
    double norm = q.norm();
    if (norm < 1e-9) {
        return Vector4d(1.0, 0.0, 0.0, 0.0);
    }
    return q / norm;
}

EKFPositionEstimator::Matrix4d EKFPositionEstimator::omegaMatrix(const Vector3d& w)
{
    Matrix4d Omega;
    Omega <<     0, -w(0), -w(1), -w(2),
              w(0),     0,  w(2), -w(1),
              w(1), -w(2),     0,  w(0),
              w(2),  w(1), -w(0),     0;
    return Omega;
}

EKFPositionEstimator::MatrixXd EKFPositionEstimator::makeSymmetric(const MatrixXd& P)
{
    return (P + P.transpose()) / 2.0;
}

// ========== 予測ステップ ==========

void EKFPositionEstimator::predict(const Vector3d& a_meas, const Vector3d& w_meas, double dt)
{
    // 現在の状態を取り出す
    Vector3d p = x_.segment<3>(0);
    Vector3d v = x_.segment<3>(3);
    Vector4d q = x_.segment<4>(6);
    Vector3d b_g = x_.segment<3>(10);
    Eigen::Vector2d b_a = x_.segment<2>(13);

    // バイアス補正
    Vector3d b_a_3d(b_a(0), b_a(1), 0.0);
    Vector3d a_B = a_meas - b_a_3d;
    Vector3d w_B = w_meas - b_g;

    // 回転行列
    Matrix3d R = quaternionToRotationMatrix(q);

    // 状態予測
    Vector3d p_pred = p + v * dt;
    Vector3d v_pred = v + (R * a_B + g_W_) * dt;
    Vector4d q_pred = q + 0.5 * omegaMatrix(w_B) * q * dt;
    q_pred = normalizeQuaternion(q_pred);
    Vector3d b_g_pred = b_g;
    Eigen::Vector2d b_a_pred = b_a;

    // 予測状態を更新
    x_.segment<3>(0) = p_pred;
    x_.segment<3>(3) = v_pred;
    x_.segment<4>(6) = q_pred;
    x_.segment<3>(10) = b_g_pred;
    x_.segment<2>(13) = b_a_pred;

    // ヤコビアン行列 F の計算
    MatrixXd F = computeStateJacobian(q, w_B, a_B, dt);

    // プロセスノイズ共分散 Q
    MatrixXd Q = computeProcessNoiseCov(dt);

    // 共分散予測
    P_ = F * P_ * F.transpose() + Q;
    P_ = makeSymmetric(P_);
}

EKFPositionEstimator::MatrixXd EKFPositionEstimator::computeStateJacobian(
    const Vector4d& q, const Vector3d& w_B, const Vector3d& a_B, double dt) const
{
    // F = I + dt * A の形式
    MatrixXd A = MatrixXd::Zero(STATE_DIM, STATE_DIM);

    // 位置の速度に関する微分: dp/dv = I
    A.block<3, 3>(0, 3) = Matrix3d::Identity();

    // 速度のクォータニオンに関する微分: dv/dq
    Matrix3d R = quaternionToRotationMatrix(q);
    A.block<3, 4>(3, 6) = computeDRaDq(q, a_B);

    // 速度の加速度バイアスに関する微分: dv/db_a = -R[:, 0:2]
    A.block<3, 2>(3, 13) = -R.block<3, 2>(0, 0);

    // クォータニオンのクォータニオンに関する微分: dq/dq = 0.5*Omega(w_B)
    A.block<4, 4>(6, 6) = 0.5 * omegaMatrix(w_B);

    // クォータニオンのジャイロバイアスに関する微分
    A.block<4, 3>(6, 10) = -0.5 * computeDOmegaDw(q);

    MatrixXd F = MatrixXd::Identity(STATE_DIM, STATE_DIM) + dt * A;
    return F;
}

Eigen::Matrix<double, 3, 4> EKFPositionEstimator::computeDRaDq(
    const Vector4d& q, const Vector3d& a) const
{
    double q_w = q(0), q_x = q(1), q_y = q(2), q_z = q(3);
    double a_x = a(0), a_y = a(1), a_z = a(2);

    Eigen::Matrix<double, 3, 4> dRa_dq;

    // ∂(R*a)/∂q_w
    dRa_dq.col(0) << 2.0 * (a_y*q_z - a_z*q_y),
                     2.0 * (a_z*q_x - a_x*q_z),
                     2.0 * (a_x*q_y - a_y*q_x);

    // ∂(R*a)/∂q_x
    dRa_dq.col(1) << 2.0 * (a_y*q_y + a_z*q_z),
                     2.0 * (a_y*q_x - 2*a_x*q_x + a_z*q_w),
                     2.0 * (a_z*q_x - a_y*q_w - 2*a_x*q_y);

    // ∂(R*a)/∂q_y
    dRa_dq.col(2) << 2.0 * (-a_x*q_y - a_z*q_w - 2*a_y*q_x),
                     2.0 * (a_x*q_x + a_z*q_z),
                     2.0 * (a_z*q_y - a_x*q_w - 2*a_y*q_z);

    // ∂(R*a)/∂q_z
    dRa_dq.col(3) << 2.0 * (-a_x*q_z + a_y*q_w - 2*a_z*q_x),
                     2.0 * (-a_y*q_z - a_x*q_w - 2*a_z*q_y),
                     2.0 * (a_x*q_x + a_y*q_y);

    return dRa_dq;
}

Eigen::Matrix<double, 4, 3> EKFPositionEstimator::computeDOmegaDw(const Vector4d& q) const
{
    double q_w = q(0), q_x = q(1), q_y = q(2), q_z = q(3);

    Eigen::Matrix<double, 4, 3> dOmega_q_dw;
    dOmega_q_dw << -q_x, -q_y, -q_z,
                    q_w,  q_z, -q_y,
                   -q_z,  q_w,  q_x,
                    q_y, -q_x,  q_w;

    return dOmega_q_dw;
}

EKFPositionEstimator::MatrixXd EKFPositionEstimator::computeProcessNoiseCov(double dt) const
{
    MatrixXd Q = MatrixXd::Zero(STATE_DIM, STATE_DIM);

    // 位置: ノイズなし（速度積分による）
    // Q.block<3, 3>(0, 0) = 0

    // 速度
    Q.block<3, 3>(3, 3) = Matrix3d::Identity() * params_.sigma_v_n * params_.sigma_v_n * dt;

    // クォータニオン
    Q.block<4, 4>(6, 6) = Matrix4d::Identity() * params_.sigma_q_n * params_.sigma_q_n * dt;

    // ジャイロバイアス
    Q.block<3, 3>(10, 10) = Matrix3d::Identity() * params_.sigma_bg_n * params_.sigma_bg_n * dt;

    // 加速度バイアス
    Q.block<2, 2>(13, 13) = Eigen::Matrix2d::Identity() * params_.sigma_ba_n * params_.sigma_ba_n * dt;

    return Q;
}

// ========== 更新ステップ ==========

void EKFPositionEstimator::updateAccelerometer(const Vector3d& z_acc)
{
    Vector4d q = x_.segment<4>(6);
    Eigen::Vector2d b_a = x_.segment<2>(13);
    Matrix3d R = quaternionToRotationMatrix(q);

    // 観測予測
    Vector3d b_a_3d(b_a(0), b_a(1), 0.0);
    Vector3d z_pred = R.transpose() * g_W_ + b_a_3d;

    // 観測ヤコビアン H
    Eigen::Matrix<double, 3, STATE_DIM> H = computeAccelerometerJacobian(q);

    // 観測ノイズ共分散
    Matrix3d R_noise = Matrix3d::Identity() * params_.sigma_acc * params_.sigma_acc;

    // カルマンゲインと更新
    kalmanUpdate(z_acc, z_pred, H, R_noise);
}

Eigen::Matrix<double, 3, EKFPositionEstimator::STATE_DIM>
EKFPositionEstimator::computeAccelerometerJacobian(const Vector4d& q) const
{
    Eigen::Matrix<double, 3, STATE_DIM> H = MatrixXd::Zero(3, STATE_DIM);

    // h_acc = R^T * g_W + [b_ax, b_ay, 0]^T
    // ∂h/∂q
    H.block<3, 4>(0, 6) = computeDRTgDq(q, g_W_);

    // ∂h/∂b_a
    H(0, 13) = 1.0;
    H(1, 14) = 1.0;
    // H(2, :) = 0（z軸バイアスは推定しない）

    return H;
}

Eigen::Matrix<double, 3, 4> EKFPositionEstimator::computeDRTgDq(
    const Vector4d& q, const Vector3d& g) const
{
    double q_w = q(0), q_x = q(1), q_y = q(2), q_z = q(3);
    double g_x = g(0), g_y = g(1), g_z = g(2);

    Eigen::Matrix<double, 3, 4> dRTg_dq;

    // 簡略化: g = [0, 0, g_z]^T の場合
    if (std::abs(g_x) < 1e-6 && std::abs(g_y) < 1e-6) {
        dRTg_dq.col(0) << 2.0*g_z*q_y, -2.0*g_z*q_z,  2.0*g_z*q_w;
        dRTg_dq.col(1) << 2.0*g_z*q_z,  2.0*g_z*q_w, -2.0*g_z*q_x;
        dRTg_dq.col(2) << 2.0*g_z*q_w,  2.0*g_z*q_x, -2.0*g_z*q_y;
        dRTg_dq.col(3) << -2.0*g_z*q_x, 2.0*g_z*q_y, -2.0*g_z*q_z;
    } else {
        // 一般的なケース（必要に応じて実装）
        dRTg_dq.setZero();
    }

    return dRTg_dq;
}

void EKFPositionEstimator::updateMagnetometer(const Vector3d& z_mag)
{
    Vector4d q = x_.segment<4>(6);
    Matrix3d R = quaternionToRotationMatrix(q);

    // 観測予測
    Vector3d z_pred = R.transpose() * m_W_;

    // 観測ヤコビアン H
    Eigen::Matrix<double, 3, STATE_DIM> H = computeMagnetometerJacobian(q);

    // 観測ノイズ共分散
    Matrix3d R_noise = Matrix3d::Identity() * params_.sigma_mag * params_.sigma_mag;

    // カルマンゲインと更新
    kalmanUpdate(z_mag, z_pred, H, R_noise);
}

Eigen::Matrix<double, 3, EKFPositionEstimator::STATE_DIM>
EKFPositionEstimator::computeMagnetometerJacobian(const Vector4d& q) const
{
    Eigen::Matrix<double, 3, STATE_DIM> H = MatrixXd::Zero(3, STATE_DIM);

    // h_mag = R^T * m_W
    // ∂h/∂q
    H.block<3, 4>(0, 6) = computeDRTgDq(q, m_W_);

    return H;
}

void EKFPositionEstimator::updateBarometer(double z_baro)
{
    double p_z = x_(2);

    // 観測予測
    double z_pred = p_z;

    // 観測ヤコビアン H (1x15)
    Eigen::Matrix<double, 1, STATE_DIM> H = MatrixXd::Zero(1, STATE_DIM);
    H(0, 2) = 1.0;  // p_z

    // 観測ノイズ共分散
    Eigen::Matrix<double, 1, 1> R_noise;
    R_noise << params_.sigma_baro * params_.sigma_baro;

    // カルマンゲインと更新
    Eigen::Matrix<double, 1, 1> z_meas_vec, z_pred_vec;
    z_meas_vec << z_baro;
    z_pred_vec << z_pred;
    kalmanUpdate(z_meas_vec, z_pred_vec, H, R_noise);
}

void EKFPositionEstimator::updateToF(double z_tof, bool use_tilt_compensation)
{
    double p_z = x_(2);
    Vector4d q = x_.segment<4>(6);

    Eigen::Matrix<double, 1, STATE_DIM> H = MatrixXd::Zero(1, STATE_DIM);
    double z_pred;

    if (use_tilt_compensation) {
        Matrix3d R = quaternionToRotationMatrix(q);
        double R_22 = R(2, 2);

        if (std::abs(R_22) < 0.1) {
            // 大きく傾いている場合はスキップ
            return;
        }

        z_pred = p_z / R_22;

        // ヤコビアン
        H(0, 2) = 1.0 / R_22;  // ∂h/∂p_z

        // ∂h/∂q = -p_z / R_22^2 * ∂R_22/∂q
        Vector4d dR22_dq(0.0, -4.0*q(1), -4.0*q(2), 0.0);
        H.block<1, 4>(0, 6) = -p_z / (R_22 * R_22) * dR22_dq.transpose();
    } else {
        // 簡易版（小傾斜近似）
        z_pred = p_z;
        H(0, 2) = 1.0;
    }

    // 観測ノイズ共分散
    Eigen::Matrix<double, 1, 1> R_noise;
    R_noise << params_.sigma_tof * params_.sigma_tof;

    // カルマンゲインと更新
    Eigen::Matrix<double, 1, 1> z_meas_vec, z_pred_vec;
    z_meas_vec << z_tof;
    z_pred_vec << z_pred;
    kalmanUpdate(z_meas_vec, z_pred_vec, H, R_noise);
}

void EKFPositionEstimator::updateOpticalFlow(const Eigen::Vector2d& z_flow, const Vector3d& w_meas)
{
    double p_z = x_(2);
    Vector3d v = x_.segment<3>(3);
    Vector4d q = x_.segment<4>(6);
    Vector3d b_g = x_.segment<3>(10);

    // 高度が低すぎる場合はスキップ
    if (p_z < 0.1) {
        return;
    }

    // バイアス補正済み角速度
    Vector3d w_B = w_meas - b_g;

    // ボディ座標系での速度
    Matrix3d R = quaternionToRotationMatrix(q);
    Vector3d v_B = R.transpose() * v;

    // 観測予測
    Eigen::Vector2d z_pred;
    z_pred(0) = (v_B(0) - w_B(1) * p_z) / p_z;
    z_pred(1) = (v_B(1) + w_B(0) * p_z) / p_z;

    // 観測ヤコビアン H (2x15)
    Eigen::Matrix<double, 2, STATE_DIM> H = computeOpticalFlowJacobian(q, v, p_z, w_B);

    // 観測ノイズ共分散
    Eigen::Matrix2d R_noise = Eigen::Matrix2d::Identity() * params_.sigma_flow * params_.sigma_flow;

    // カルマンゲインと更新
    kalmanUpdate(z_flow, z_pred, H, R_noise);
}

Eigen::Matrix<double, 2, EKFPositionEstimator::STATE_DIM>
EKFPositionEstimator::computeOpticalFlowJacobian(
    const Vector4d& q, const Vector3d& v, double p_z, const Vector3d& w_B) const
{
    Eigen::Matrix<double, 2, STATE_DIM> H = MatrixXd::Zero(2, STATE_DIM);

    Matrix3d R = quaternionToRotationMatrix(q);
    Vector3d v_B = R.transpose() * v;

    // ∂h/∂p_z
    H(0, 2) = -(v_B(0) - w_B(1)*p_z) / (p_z * p_z);
    H(1, 2) = -(v_B(1) + w_B(0)*p_z) / (p_z * p_z);

    // ∂h/∂v = (1/p_z) * ∂v_B/∂v = (1/p_z) * R^T
    H.block<2, 3>(0, 3) = (1.0 / p_z) * R.transpose().block<2, 3>(0, 0);

    // ∂h/∂q: v_Bのqに関する微分
    Eigen::Matrix<double, 3, 4> dv_B_dq = computeDRTvDq(q, v);
    H.block<2, 4>(0, 6) = (1.0 / p_z) * dv_B_dq.block<2, 4>(0, 0);

    // ∂h/∂b_g: 角速度バイアスの影響
    H(0, 11) = 1.0;   // b_gy
    H(1, 10) = -1.0;  // b_gx

    return H;
}

Eigen::Matrix<double, 3, 4> EKFPositionEstimator::computeDRTvDq(
    const Vector4d& q, const Vector3d& v) const
{
    double q_w = q(0), q_x = q(1), q_y = q(2), q_z = q(3);
    double v_x = v(0), v_y = v(1), v_z = v(2);

    Eigen::Matrix<double, 3, 4> dRTv_dq;

    // ∂(R^T*v)/∂q_w
    dRTv_dq.col(0) << 2.0 * (v_y*q_z - v_z*q_y),
                      2.0 * (v_z*q_x - v_x*q_z),
                      2.0 * (v_x*q_y - v_y*q_x);

    // ∂(R^T*v)/∂q_x (簡易実装)
    dRTv_dq.col(1) << 2.0 * (v_x*q_x + v_y*q_y - 2*v_x*(q_y*q_y + q_z*q_z) + v_z*q_w),
                      2.0 * (v_y*q_x - 2*v_y*q_x*q_x + v_z*q_z),
                      2.0 * (v_z*q_x - v_y*q_w - 2*v_z*q_x*q_x);

    // 残りは簡易的に0（完全実装は省略）
    dRTv_dq.col(2).setZero();
    dRTv_dq.col(3).setZero();

    return dRTv_dq;
}

void EKFPositionEstimator::kalmanUpdate(const VectorXd& z_meas, const VectorXd& z_pred,
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

    // 状態更新
    x_ = x_ + K * y;

    // クォータニオンを正規化
    x_.segment<4>(6) = normalizeQuaternion(x_.segment<4>(6));

    // 共分散更新（Joseph形式で数値安定性向上）
    MatrixXd I_KH = MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    P_ = makeSymmetric(P_);
}

// ========== 状態取得 ==========

EKFPositionEstimator::Vector3d EKFPositionEstimator::getEulerAngles() const
{
    Vector4d q = x_.segment<4>(6);
    double q_w = q(0), q_x = q(1), q_y = q(2), q_z = q(3);

    // ロール（x軸周り）
    double roll = std::atan2(2*(q_w*q_x + q_y*q_z), 1 - 2*(q_x*q_x + q_y*q_y));

    // ピッチ（y軸周り）
    double pitch = std::asin(2*(q_w*q_y - q_z*q_x));

    // ヨー（z軸周り）
    double yaw = std::atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y*q_y + q_z*q_z));

    return Vector3d(roll, pitch, yaw);
}

EKFPositionEstimator::Vector3d EKFPositionEstimator::getPositionStd() const
{
    return Vector3d(std::sqrt(P_(0, 0)), std::sqrt(P_(1, 1)), std::sqrt(P_(2, 2)));
}

}  // namespace quadcopter
