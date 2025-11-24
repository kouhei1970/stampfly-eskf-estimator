/**
 * @file ekf_position_estimator.hpp
 * @brief クアッドコプター位置・姿勢推定 EKF実装（C++版）
 *
 * 状態ベクトル (15次元):
 *   x = [p_x, p_y, p_z, v_x, v_y, v_z, q_w, q_x, q_y, q_z, b_gx, b_gy, b_gz, b_ax, b_ay]^T
 */

#ifndef EKF_POSITION_ESTIMATOR_HPP
#define EKF_POSITION_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <cmath>

namespace quadcopter {

/**
 * @brief EKFパラメータ構造体
 */
struct EKFParameters {
    // プロセスノイズ（連続時間）
    double sigma_v_n = 0.1;      // 速度プロセスノイズ [m/s²]
    double sigma_q_n = 0.001;    // 姿勢プロセスノイズ [rad/√s]
    double sigma_bg_n = 0.0001;  // ジャイロバイアスランダムウォーク [rad/s/√s]
    double sigma_ba_n = 0.001;   // 加速度バイアスランダムウォーク [m/s²/√s]

    // 観測ノイズ
    double sigma_acc = 0.1;      // 加速度センサノイズ [m/s²]
    double sigma_mag = 0.3;      // 地磁気センサノイズ [正規化単位]
    double sigma_baro = 1.0;     // 気圧高度ノイズ [m]
    double sigma_tof = 0.1;      // ToF高度ノイズ [m]
    double sigma_flow = 1.0;     // オプティカルフローノイズ [rad/s]

    // 初期共分散
    double sigma_p_init = 1.0;   // 位置 [m]
    double sigma_v_init = 0.5;   // 速度 [m/s]
    double sigma_q_init = 0.1;   // 姿勢 [rad]
    double sigma_bg_init = 0.01; // ジャイロバイアス [rad/s]
    double sigma_ba_init = 0.1;  // 加速度バイアス [m/s²]

    // 物理定数
    double g = 9.81;  // 重力加速度 [m/s²]

    // 地磁気ベクトル（ワールド座標系、正規化済み）
    double mag_inclination = 49.0 * M_PI / 180.0;  // 伏角 [rad]（日本: 約49度）
    double m_N = std::cos(mag_inclination);        // 北成分
    double m_D = std::sin(mag_inclination);        // 下成分

    // 外れ値検出
    double mahalanobis_threshold = 9.21;  // χ² threshold (3-DOF, 95%)
};

/**
 * @brief EKFベース位置・姿勢推定器クラス
 */
class EKFPositionEstimator {
public:
    static constexpr int STATE_DIM = 15;  // 状態次元

    using Vector3d = Eigen::Vector3d;
    using Vector4d = Eigen::Vector4d;
    using VectorXd = Eigen::VectorXd;
    using MatrixXd = Eigen::MatrixXd;
    using Matrix3d = Eigen::Matrix3d;
    using Matrix4d = Eigen::Matrix4d;

    /**
     * @brief コンストラクタ
     * @param params EKFパラメータ
     */
    explicit EKFPositionEstimator(const EKFParameters& params = EKFParameters());

    /**
     * @brief 予測ステップ（IMU測定値を使用）
     * @param a_meas 加速度測定値 [m/s²] (ボディ座標系)
     * @param w_meas 角速度測定値 [rad/s] (ボディ座標系)
     * @param dt サンプリング時間 [s]
     */
    void predict(const Vector3d& a_meas, const Vector3d& w_meas, double dt);

    /**
     * @brief 加速度センサによる更新
     * @param z_acc 加速度測定値 [m/s²] (ボディ座標系)
     */
    void updateAccelerometer(const Vector3d& z_acc);

    /**
     * @brief 地磁気センサによる更新
     * @param z_mag 地磁気測定値（正規化済み）(ボディ座標系)
     */
    void updateMagnetometer(const Vector3d& z_mag);

    /**
     * @brief 気圧高度センサによる更新
     * @param z_baro 気圧高度測定値 [m]
     */
    void updateBarometer(double z_baro);

    /**
     * @brief ToF高度センサによる更新
     * @param z_tof ToF高度測定値 [m]
     * @param use_tilt_compensation 傾斜補正を使用するか
     */
    void updateToF(double z_tof, bool use_tilt_compensation = false);

    /**
     * @brief オプティカルフローによる更新
     * @param z_flow オプティカルフロー測定値 [rad/s]
     * @param w_meas 角速度測定値 [rad/s] (ボディ座標系)
     */
    void updateOpticalFlow(const Eigen::Vector2d& z_flow, const Vector3d& w_meas);

    // ========== 状態取得 ==========

    /**
     * @brief 位置を取得 [m]
     */
    Vector3d getPosition() const { return x_.segment<3>(0); }

    /**
     * @brief 速度を取得 [m/s]
     */
    Vector3d getVelocity() const { return x_.segment<3>(3); }

    /**
     * @brief 姿勢クォータニオンを取得
     */
    Vector4d getQuaternion() const { return x_.segment<4>(6); }

    /**
     * @brief オイラー角を取得（ロール、ピッチ、ヨー）[rad]
     */
    Vector3d getEulerAngles() const;

    /**
     * @brief ジャイロバイアスを取得 [rad/s]
     */
    Vector3d getGyroBias() const { return x_.segment<3>(10); }

    /**
     * @brief 加速度バイアスを取得 [m/s²]
     */
    Eigen::Vector2d getAccelBias() const { return x_.segment<2>(13); }

    /**
     * @brief 誤差共分散行列を取得
     */
    const MatrixXd& getCovariance() const { return P_; }

    /**
     * @brief 位置の標準偏差を取得 [m]
     */
    Vector3d getPositionStd() const;

    /**
     * @brief 完全な状態ベクトルを取得
     */
    const VectorXd& getState() const { return x_; }

private:
    // ========== ユーティリティ関数 ==========

    /**
     * @brief クォータニオンから回転行列への変換
     * @param q クォータニオン [q_w, q_x, q_y, q_z]
     * @return 3x3回転行列（ボディ→ワールド座標系）
     */
    static Matrix3d quaternionToRotationMatrix(const Vector4d& q);

    /**
     * @brief クォータニオンを正規化
     */
    static Vector4d normalizeQuaternion(const Vector4d& q);

    /**
     * @brief クォータニオン微分用のΩ行列
     * @param w 角速度ベクトル [w_x, w_y, w_z]
     * @return 4x4行列
     */
    static Matrix4d omegaMatrix(const Vector3d& w);

    /**
     * @brief 共分散行列の対称性を強制
     */
    static MatrixXd makeSymmetric(const MatrixXd& P);

    // ========== ヤコビアン計算 ==========

    /**
     * @brief 状態遷移のヤコビアン行列 F を計算
     */
    MatrixXd computeStateJacobian(const Vector4d& q, const Vector3d& w_B,
                                   const Vector3d& a_B, double dt) const;

    /**
     * @brief R(q) * a のクォータニオンに関する微分を計算
     */
    Eigen::Matrix<double, 3, 4> computeDRaDq(const Vector4d& q, const Vector3d& a) const;

    /**
     * @brief ∂(Ω(ω)*q)/∂ω を計算
     */
    Eigen::Matrix<double, 4, 3> computeDOmegaDw(const Vector4d& q) const;

    /**
     * @brief プロセスノイズ共分散行列 Q を計算
     */
    MatrixXd computeProcessNoiseCov(double dt) const;

    /**
     * @brief 加速度センサの観測ヤコビアン H を計算
     */
    Eigen::Matrix<double, 3, STATE_DIM> computeAccelerometerJacobian(const Vector4d& q) const;

    /**
     * @brief R^T * g のクォータニオンに関する微分
     */
    Eigen::Matrix<double, 3, 4> computeDRTgDq(const Vector4d& q, const Vector3d& g) const;

    /**
     * @brief 地磁気センサの観測ヤコビアン H を計算
     */
    Eigen::Matrix<double, 3, STATE_DIM> computeMagnetometerJacobian(const Vector4d& q) const;

    /**
     * @brief オプティカルフローの観測ヤコビアン H を計算
     */
    Eigen::Matrix<double, 2, STATE_DIM> computeOpticalFlowJacobian(
        const Vector4d& q, const Vector3d& v, double p_z, const Vector3d& w_B) const;

    /**
     * @brief R^T * v のクォータニオンに関する微分
     */
    Eigen::Matrix<double, 3, 4> computeDRTvDq(const Vector4d& q, const Vector3d& v) const;

    // ========== 更新処理 ==========

    /**
     * @brief カルマンゲインによる状態・共分散更新
     */
    void kalmanUpdate(const VectorXd& z_meas, const VectorXd& z_pred,
                      const MatrixXd& H, const MatrixXd& R);

    /**
     * @brief 初期共分散行列を生成
     */
    MatrixXd initializeCovariance() const;

    // ========== メンバ変数 ==========

    EKFParameters params_;       // パラメータ
    VectorXd x_;                 // 状態ベクトル (15次元)
    MatrixXd P_;                 // 誤差共分散行列 (15x15)
    Vector3d m_W_;               // 地磁気ベクトル（ワールド座標系）
    Vector3d g_W_;               // 重力ベクトル（ワールド座標系）
};

}  // namespace quadcopter

#endif  // EKF_POSITION_ESTIMATOR_HPP
