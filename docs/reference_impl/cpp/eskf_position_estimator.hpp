/**
 * @file eskf_position_estimator.hpp
 * @brief クアッドコプター位置・姿勢推定 ESKF実装（C++版）
 *
 * Error-State Kalman Filter (ESKF) implementation
 *
 * ノミナル状態 (15次元):
 *   x = [p_x, p_y, p_z, v_x, v_y, v_z, q_w, q_x, q_y, q_z, b_gx, b_gy, b_gz, b_ax, b_ay]^T
 *
 * エラー状態 (15次元):
 *   δx = [δp_x, δp_y, δp_z, δv_x, δv_y, δv_z, δθ_x, δθ_y, δθ_z, δb_gx, δb_gy, δb_gz, δb_ax, δb_ay]^T
 *   注: 姿勢誤差は3次元の回転ベクトル δθ で表現
 */

#ifndef ESKF_POSITION_ESTIMATOR_HPP
#define ESKF_POSITION_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <cmath>

namespace quadcopter {

/**
 * @brief ESKFパラメータ構造体
 */
struct ESKFParameters {
    // プロセスノイズ（連続時間）
    double sigma_a_n = 0.1;       // 加速度ノイズ [m/s²]
    double sigma_omega_n = 0.001; // 角速度ノイズ [rad/s]
    double sigma_bg_n = 0.0001;   // ジャイロバイアスランダムウォーク [rad/s/√s]
    double sigma_ba_n = 0.001;    // 加速度バイアスランダムウォーク [m/s²/√s]

    // 観測ノイズ
    double sigma_acc = 0.1;      // 加速度センサノイズ [m/s²]
    double sigma_mag = 0.3;      // 地磁気センサノイズ [正規化単位]
    double sigma_baro = 1.0;     // 気圧高度ノイズ [m]
    double sigma_tof = 0.1;      // ToF高度ノイズ [m]
    double sigma_flow = 1.0;     // オプティカルフローノイズ [rad/s]

    // 初期共分散
    double sigma_p_init = 1.0;      // 位置 [m]
    double sigma_v_init = 0.5;      // 速度 [m/s]
    double sigma_theta_init = 0.1;  // 姿勢（回転ベクトル） [rad]
    double sigma_bg_init = 0.01;    // ジャイロバイアス [rad/s]
    double sigma_ba_init = 0.1;     // 加速度バイアス [m/s²]

    // 物理定数
    double g = 9.81;  // 重力加速度 [m/s²]

    // 地磁気ベクトル（ワールド座標系、正規化済み）
    double mag_inclination = 49.0 * M_PI / 180.0;  // 伏角 [rad]
    double m_N = std::cos(mag_inclination);        // 北成分
    double m_D = std::sin(mag_inclination);        // 下成分

    // 外れ値検出
    double mahalanobis_threshold = 9.21;  // χ² threshold (3-DOF, 95%)
};

/**
 * @brief ESKFベース位置・姿勢推定器クラス
 */
class ESKFPositionEstimator {
public:
    static constexpr int STATE_DIM = 15;       // ノミナル状態次元
    static constexpr int ERROR_STATE_DIM = 15; // エラー状態次元

    using Vector3d = Eigen::Vector3d;
    using Vector4d = Eigen::Vector4d;
    using VectorXd = Eigen::VectorXd;
    using MatrixXd = Eigen::MatrixXd;
    using Matrix3d = Eigen::Matrix3d;
    using Matrix4d = Eigen::Matrix4d;

    /**
     * @brief コンストラクタ
     * @param params ESKFパラメータ
     */
    explicit ESKFPositionEstimator(const ESKFParameters& params = ESKFParameters());

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
    Vector3d getPosition() const { return x_nominal_.segment<3>(0); }

    /**
     * @brief 速度を取得 [m/s]
     */
    Vector3d getVelocity() const { return x_nominal_.segment<3>(3); }

    /**
     * @brief 姿勢クォータニオンを取得
     */
    Vector4d getQuaternion() const { return x_nominal_.segment<4>(6); }

    /**
     * @brief オイラー角を取得（ロール、ピッチ、ヨー）[rad]
     */
    Vector3d getEulerAngles() const;

    /**
     * @brief ジャイロバイアスを取得 [rad/s]
     */
    Vector3d getGyroBias() const { return x_nominal_.segment<3>(10); }

    /**
     * @brief 加速度バイアスを取得 [m/s²]
     */
    Eigen::Vector2d getAccelBias() const { return x_nominal_.segment<2>(13); }

    /**
     * @brief エラー状態共分散行列を取得
     */
    const MatrixXd& getErrorCovariance() const { return P_; }

    /**
     * @brief 位置の標準偏差を取得 [m]
     */
    Vector3d getPositionStd() const;

    /**
     * @brief ノミナル状態ベクトルを取得
     */
    const VectorXd& getNominalState() const { return x_nominal_; }

    /**
     * @brief エラー状態ベクトルを取得（通常は0）
     */
    const VectorXd& getErrorState() const { return dx_error_; }

private:
    // ========== ユーティリティ関数 ==========

    /**
     * @brief クォータニオンから回転行列への変換
     */
    static Matrix3d quaternionToRotationMatrix(const Vector4d& q);

    /**
     * @brief クォータニオンを正規化
     */
    static Vector4d normalizeQuaternion(const Vector4d& q);

    /**
     * @brief クォータニオン微分用のΩ行列
     */
    static Matrix4d omegaMatrix(const Vector3d& w);

    /**
     * @brief 歪対称行列（skew-symmetric matrix）を生成
     * @param v 3次元ベクトル
     * @return [v]_× (3x3行列)
     */
    static Matrix3d skewSymmetric(const Vector3d& v);

    /**
     * @brief 回転ベクトルからクォータニオンへの変換（小角度近似）
     * @param delta_theta 回転ベクトル [rad]
     * @return クォータニオン
     */
    static Vector4d smallAngleQuaternion(const Vector3d& delta_theta);

    /**
     * @brief 共分散行列の対称性を強制
     */
    static MatrixXd makeSymmetric(const MatrixXd& P);

    // ========== 予測関連 ==========

    /**
     * @brief ノミナル状態を伝播
     */
    void propagateNominalState(const Vector3d& a_meas, const Vector3d& w_meas, double dt);

    /**
     * @brief 連続時間システム行列 F_c を計算
     */
    MatrixXd computeContinuousErrorStateJacobian(const Vector4d& q,
                                                  const Vector3d& a_corrected,
                                                  const Vector3d& w_corrected) const;

    /**
     * @brief プロセスノイズ共分散行列 Q_d を計算
     */
    MatrixXd computeProcessNoiseCov(double dt) const;

    // ========== 更新関連 ==========

    /**
     * @brief 加速度センサの観測ヤコビアン H を計算
     */
    Eigen::Matrix<double, 3, ERROR_STATE_DIM> computeAccelerometerJacobian(const Vector4d& q) const;

    /**
     * @brief 地磁気センサの観測ヤコビアン H を計算
     */
    Eigen::Matrix<double, 3, ERROR_STATE_DIM> computeMagnetometerJacobian(const Vector4d& q) const;

    /**
     * @brief オプティカルフローの観測ヤコビアン H を計算
     */
    Eigen::Matrix<double, 2, ERROR_STATE_DIM> computeOpticalFlowJacobian(
        const Vector4d& q, const Vector3d& v, double p_z, const Vector3d& w_corrected) const;

    /**
     * @brief カルマンゲインによる更新とエラー状態のリセット
     */
    void kalmanUpdateAndReset(const VectorXd& z_meas, const VectorXd& z_pred,
                               const MatrixXd& H, const MatrixXd& R);

    /**
     * @brief エラー状態をノミナル状態に注入（inject）
     * @param dx エラー状態
     */
    void injectErrorState(const VectorXd& dx);

    /**
     * @brief 初期共分散行列を生成
     */
    MatrixXd initializeCovariance() const;

    // ========== メンバ変数 ==========

    ESKFParameters params_;      // パラメータ

    // ノミナル状態 (15次元)
    VectorXd x_nominal_;

    // エラー状態 (15次元)（通常は0、更新後すぐにリセット）
    VectorXd dx_error_;

    // エラー状態共分散行列 (15x15)
    MatrixXd P_;

    Vector3d m_W_;               // 地磁気ベクトル（ワールド座標系）
    Vector3d g_W_;               // 重力ベクトル（ワールド座標系）
};

}  // namespace quadcopter

#endif  // ESKF_POSITION_ESTIMATOR_HPP
