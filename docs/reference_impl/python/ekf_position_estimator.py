"""
クアッドコプター位置・姿勢推定 EKF実装
EKF_Position_Estimation.md の数式を実装

状態ベクトル (15次元):
    x = [p_x, p_y, p_z, v_x, v_y, v_z, q_w, q_x, q_y, q_z, b_gx, b_gy, b_gz, b_ax, b_ay]^T
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class EKFParameters:
    """EKFパラメータ"""
    # プロセスノイズ（連続時間）
    sigma_v_n: float = 0.1      # 速度プロセスノイズ [m/s²]
    sigma_q_n: float = 0.001    # 姿勢プロセスノイズ [rad/√s]
    sigma_bg_n: float = 0.0001  # ジャイロバイアスランダムウォーク [rad/s/√s]
    sigma_ba_n: float = 0.001   # 加速度バイアスランダムウォーク [m/s²/√s]

    # 観測ノイズ
    sigma_acc: float = 0.1      # 加速度センサノイズ [m/s²]
    sigma_mag: float = 0.3      # 地磁気センサノイズ [正規化単位]
    sigma_baro: float = 1.0     # 気圧高度ノイズ [m]
    sigma_tof: float = 0.1      # ToF高度ノイズ [m]
    sigma_flow: float = 1.0     # オプティカルフローノイズ [rad/s]

    # 初期共分散
    sigma_p_init: float = 1.0       # 位置 [m]
    sigma_v_init: float = 0.5       # 速度 [m/s]
    sigma_q_init: float = 0.1       # 姿勢 [rad]
    sigma_bg_init: float = 0.01     # ジャイロバイアス [rad/s]
    sigma_ba_init: float = 0.1      # 加速度バイアス [m/s²]

    # 物理定数
    g: float = 9.81  # 重力加速度 [m/s²]

    # 地磁気ベクトル（ワールド座標系、正規化済み）
    # 日本の場合、伏角約49度
    mag_inclination: float = 49.0 * np.pi / 180.0  # [rad]
    m_N: float = None  # 北成分（自動計算）
    m_D: float = None  # 下成分（自動計算）

    def __post_init__(self):
        """地磁気ベクトル成分を計算"""
        if self.m_N is None:
            self.m_N = np.cos(self.mag_inclination)
        if self.m_D is None:
            self.m_D = np.sin(self.mag_inclination)


class EKFPositionEstimator:
    """EKFベース位置・姿勢推定器"""

    def __init__(self, params: Optional[EKFParameters] = None):
        """
        初期化

        Args:
            params: EKFパラメータ。Noneの場合はデフォルト値を使用
        """
        self.params = params if params is not None else EKFParameters()

        # 状態ベクトル (15次元)
        self.x = np.zeros(15)
        self.x[6] = 1.0  # クォータニオン q_w = 1（単位クォータニオン）

        # 誤差共分散行列 (15x15)
        self.P = self._initialize_covariance()

        # 地磁気ベクトル（ワールド座標系）
        self.m_W = np.array([self.params.m_N, 0.0, self.params.m_D])

        # 重力ベクトル（ワールド座標系、NED座標系）
        self.g_W = np.array([0.0, 0.0, self.params.g])

    def _initialize_covariance(self) -> np.ndarray:
        """初期共分散行列を生成"""
        P = np.eye(15)
        # 位置
        P[0:3, 0:3] *= self.params.sigma_p_init ** 2
        # 速度
        P[3:6, 3:6] *= self.params.sigma_v_init ** 2
        # クォータニオン
        P[6:10, 6:10] *= self.params.sigma_q_init ** 2
        # ジャイロバイアス
        P[10:13, 10:13] *= self.params.sigma_bg_init ** 2
        # 加速度バイアス
        P[13:15, 13:15] *= self.params.sigma_ba_init ** 2
        return P

    # ========== ユーティリティ関数 ==========

    @staticmethod
    def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """
        クォータニオンから回転行列への変換

        Args:
            q: クォータニオン [q_w, q_x, q_y, q_z]

        Returns:
            R: 3x3回転行列（ボディ→ワールド座標系）
        """
        q_w, q_x, q_y, q_z = q[0], q[1], q[2], q[3]

        R = np.array([
            [1 - 2*(q_y**2 + q_z**2),     2*(q_x*q_y - q_w*q_z),     2*(q_x*q_z + q_w*q_y)],
            [    2*(q_x*q_y + q_w*q_z), 1 - 2*(q_x**2 + q_z**2),     2*(q_y*q_z - q_w*q_x)],
            [    2*(q_x*q_z - q_w*q_y),     2*(q_y*q_z + q_w*q_x), 1 - 2*(q_x**2 + q_y**2)]
        ])
        return R

    @staticmethod
    def normalize_quaternion(q: np.ndarray) -> np.ndarray:
        """クォータニオンを正規化"""
        norm = np.linalg.norm(q)
        if norm < 1e-9:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm

    @staticmethod
    def omega_matrix(w: np.ndarray) -> np.ndarray:
        """
        クォータニオン微分用のΩ行列

        Args:
            w: 角速度ベクトル [w_x, w_y, w_z]

        Returns:
            Omega: 4x4行列
        """
        w_x, w_y, w_z = w[0], w[1], w[2]
        Omega = np.array([
            [   0,  -w_x,  -w_y,  -w_z],
            [ w_x,     0,   w_z,  -w_y],
            [ w_y,  -w_z,     0,   w_x],
            [ w_z,   w_y,  -w_x,     0]
        ])
        return Omega

    def _make_symmetric(self, P: np.ndarray) -> np.ndarray:
        """共分散行列の対称性を強制"""
        return (P + P.T) / 2.0

    # ========== 予測ステップ ==========

    def predict(self, a_meas: np.ndarray, w_meas: np.ndarray, dt: float):
        """
        予測ステップ（IMU測定値を使用）

        Args:
            a_meas: 加速度測定値 [a_x, a_y, a_z] (ボディ座標系) [m/s²]
            w_meas: 角速度測定値 [w_x, w_y, w_z] (ボディ座標系) [rad/s]
            dt: サンプリング時間 [s]
        """
        # 現在の状態を取り出す
        p = self.x[0:3]
        v = self.x[3:6]
        q = self.x[6:10]
        b_g = self.x[10:13]
        b_a = self.x[13:15]

        # バイアス補正
        b_a_3d = np.array([b_a[0], b_a[1], 0.0])
        a_B = a_meas - b_a_3d
        w_B = w_meas - b_g

        # 回転行列
        R = self.quaternion_to_rotation_matrix(q)

        # 状態予測
        p_pred = p + v * dt
        v_pred = v + (R @ a_B + self.g_W) * dt
        q_pred = q + 0.5 * self.omega_matrix(w_B) @ q * dt
        q_pred = self.normalize_quaternion(q_pred)
        b_g_pred = b_g
        b_a_pred = b_a

        # 予測状態を更新
        x_pred = np.concatenate([p_pred, v_pred, q_pred, b_g_pred, b_a_pred])

        # ヤコビアン行列 F の計算
        F = self._compute_state_jacobian(q, w_B, a_B, dt)

        # プロセスノイズ共分散 Q
        Q = self._compute_process_noise_covariance(dt)

        # 共分散予測
        P_pred = F @ self.P @ F.T + Q
        P_pred = self._make_symmetric(P_pred)

        # 状態と共分散を更新
        self.x = x_pred
        self.P = P_pred

    def _compute_state_jacobian(self, q: np.ndarray, w_B: np.ndarray,
                                 a_B: np.ndarray, dt: float) -> np.ndarray:
        """
        状態遷移のヤコビアン行列 F を計算

        Args:
            q: クォータニオン
            w_B: バイアス補正済み角速度
            a_B: バイアス補正済み加速度
            dt: サンプリング時間

        Returns:
            F: 15x15ヤコビアン行列
        """
        # F = I + dt * A の形式
        A = np.zeros((15, 15))

        # 位置の速度に関する微分: dp/dv = I
        A[0:3, 3:6] = np.eye(3)

        # 速度のクォータニオンに関する微分: dv/dq
        R = self.quaternion_to_rotation_matrix(q)
        dv_dq = self._compute_dRa_dq(q, a_B)
        A[3:6, 6:10] = dv_dq

        # 速度の加速度バイアスに関する微分: dv/db_a = -R[:, 0:2]
        A[3:6, 13:15] = -R[:, 0:2]

        # クォータニオンのクォータニオンに関する微分: dq/dq = 0.5*Omega(w_B)
        A[6:10, 6:10] = 0.5 * self.omega_matrix(w_B)

        # クォータニオンのジャイロバイアスに関する微分
        # dq̇/db_g = -0.5 * ∂Ω/∂ω * q
        dq_dbg = -0.5 * self._compute_dOmega_dw(q)
        A[6:10, 10:13] = dq_dbg

        # バイアスは定数（ランダムウォーク）
        # A[10:13, 10:13] = 0
        # A[13:15, 13:15] = 0

        F = np.eye(15) + dt * A
        return F

    def _compute_dRa_dq(self, q: np.ndarray, a: np.ndarray) -> np.ndarray:
        """
        R(q) * a のクォータニオンに関する微分を計算

        Args:
            q: クォータニオン [q_w, q_x, q_y, q_z]
            a: ベクトル [a_x, a_y, a_z]

        Returns:
            dRa_dq: 3x4行列
        """
        q_w, q_x, q_y, q_z = q[0], q[1], q[2], q[3]
        a_x, a_y, a_z = a[0], a[1], a[2]

        # ∂(R*a)/∂q_w
        dRa_dq_w = 2.0 * np.array([
            a_y*q_z - a_z*q_y,
            a_z*q_x - a_x*q_z,
            a_x*q_y - a_y*q_x
        ])

        # ∂(R*a)/∂q_x
        dRa_dq_x = 2.0 * np.array([
            a_y*q_y + a_z*q_z,
            a_y*q_x - 2*a_x*q_x + a_z*q_w,
            a_z*q_x - a_y*q_w - 2*a_x*q_y
        ])

        # ∂(R*a)/∂q_y
        dRa_dq_y = 2.0 * np.array([
            -a_x*q_y - a_z*q_w - 2*a_y*q_x,
            a_x*q_x + a_z*q_z,
            a_z*q_y - a_x*q_w - 2*a_y*q_z
        ])

        # ∂(R*a)/∂q_z
        dRa_dq_z = 2.0 * np.array([
            -a_x*q_z + a_y*q_w - 2*a_z*q_x,
            -a_y*q_z - a_x*q_w - 2*a_z*q_y,
            a_x*q_x + a_y*q_y
        ])

        dRa_dq = np.column_stack([dRa_dq_w, dRa_dq_x, dRa_dq_y, dRa_dq_z])
        return dRa_dq

    def _compute_dOmega_dw(self, q: np.ndarray) -> np.ndarray:
        """
        ∂(Ω(ω)*q)/∂ω を計算（簡易版：各成分ごと）

        Returns:
            dOmega_q_dw: 4x3行列
        """
        q_w, q_x, q_y, q_z = q[0], q[1], q[2], q[3]

        # ∂(Ω*q)/∂ω_x, ∂(Ω*q)/∂ω_y, ∂(Ω*q)/∂ω_z
        dOmega_q_dw = np.array([
            [-q_x, -q_y, -q_z],
            [ q_w,  q_z, -q_y],
            [-q_z,  q_w,  q_x],
            [ q_y, -q_x,  q_w]
        ])
        return dOmega_q_dw

    def _compute_process_noise_covariance(self, dt: float) -> np.ndarray:
        """プロセスノイズ共分散行列 Q を計算"""
        Q = np.zeros((15, 15))

        # 位置: ノイズなし（速度積分による）
        # Q[0:3, 0:3] = 0

        # 速度
        Q[3:6, 3:6] = (self.params.sigma_v_n ** 2) * dt * np.eye(3)

        # クォータニオン
        Q[6:10, 6:10] = (self.params.sigma_q_n ** 2) * dt * np.eye(4)

        # ジャイロバイアス
        Q[10:13, 10:13] = (self.params.sigma_bg_n ** 2) * dt * np.eye(3)

        # 加速度バイアス
        Q[13:15, 13:15] = (self.params.sigma_ba_n ** 2) * dt * np.eye(2)

        return Q

    # ========== 更新ステップ ==========

    def update_accelerometer(self, z_acc: np.ndarray):
        """
        加速度センサによる更新

        Args:
            z_acc: 加速度測定値 [a_x, a_y, a_z] (ボディ座標系) [m/s²]
        """
        # 観測予測
        q = self.x[6:10]
        b_a = self.x[13:15]
        R = self.quaternion_to_rotation_matrix(q)

        # 期待される測定値（静止時、重力のみ）
        b_a_3d = np.array([b_a[0], b_a[1], 0.0])
        z_pred = R.T @ self.g_W + b_a_3d

        # 観測ヤコビアン H (3x15)
        H = self._compute_accelerometer_jacobian(q)

        # 観測ノイズ共分散
        R_noise = (self.params.sigma_acc ** 2) * np.eye(3)

        # カルマンゲインと更新
        self._kalman_update(z_acc, z_pred, H, R_noise)

    def _compute_accelerometer_jacobian(self, q: np.ndarray) -> np.ndarray:
        """加速度センサの観測ヤコビアン H を計算"""
        H = np.zeros((3, 15))

        # h_acc = R^T * g_W + [b_ax, b_ay, 0]^T
        # ∂h/∂q
        dRTg_dq = self._compute_dRTg_dq(q, self.g_W)
        H[0:3, 6:10] = dRTg_dq

        # ∂h/∂b_a
        H[0, 13] = 1.0
        H[1, 14] = 1.0
        # H[2, :] = 0（z軸バイアスは推定しない）

        return H

    def _compute_dRTg_dq(self, q: np.ndarray, g: np.ndarray) -> np.ndarray:
        """
        R(q)^T * g のクォータニオンに関する微分

        Args:
            q: クォータニオン
            g: ベクトル（通常は重力）

        Returns:
            dRTg_dq: 3x4行列
        """
        q_w, q_x, q_y, q_z = q[0], q[1], q[2], q[3]
        g_x, g_y, g_z = g[0], g[1], g[2]

        # 簡略化: g = [0, 0, g_z]^T の場合
        if abs(g_x) < 1e-6 and abs(g_y) < 1e-6:
            dRTg_dq = 2.0 * g_z * np.array([
                [ q_y, -q_z,  q_w, -q_x],
                [ q_z,  q_w,  q_x, -q_y],
                [ q_w,  q_x,  q_y,  q_z]
            ])
        else:
            # 一般的なケース（地磁気など）
            # R^T の各成分の微分を計算（省略：必要に応じて実装）
            dRTg_dq = np.zeros((3, 4))
            # TODO: 一般的なケースの実装

        return dRTg_dq

    def update_magnetometer(self, z_mag: np.ndarray):
        """
        地磁気センサによる更新

        Args:
            z_mag: 地磁気測定値 [m_x, m_y, m_z] (ボディ座標系、正規化済み)
        """
        # 観測予測
        q = self.x[6:10]
        R = self.quaternion_to_rotation_matrix(q)
        z_pred = R.T @ self.m_W

        # 観測ヤコビアン H (3x15)
        H = self._compute_magnetometer_jacobian(q)

        # 観測ノイズ共分散
        R_noise = (self.params.sigma_mag ** 2) * np.eye(3)

        # カルマンゲインと更新
        self._kalman_update(z_mag, z_pred, H, R_noise)

    def _compute_magnetometer_jacobian(self, q: np.ndarray) -> np.ndarray:
        """地磁気センサの観測ヤコビアン H を計算"""
        H = np.zeros((3, 15))

        # h_mag = R^T * m_W
        # ∂h/∂q
        dRTm_dq = self._compute_dRTg_dq(q, self.m_W)
        H[0:3, 6:10] = dRTm_dq

        return H

    def update_barometer(self, z_baro: float):
        """
        気圧高度センサによる更新

        Args:
            z_baro: 気圧高度測定値 [m]（下方向正）
        """
        # 観測予測
        p_z = self.x[2]
        z_pred = p_z

        # 観測ヤコビアン H (1x15)
        H = np.zeros((1, 15))
        H[0, 2] = 1.0  # p_z

        # 観測ノイズ共分散
        R_noise = np.array([[self.params.sigma_baro ** 2]])

        # カルマンゲインと更新
        z_meas = np.array([z_baro])
        z_pred_arr = np.array([z_pred])
        self._kalman_update(z_meas, z_pred_arr, H, R_noise)

    def update_tof(self, z_tof: float, use_tilt_compensation: bool = False):
        """
        ToF高度センサによる更新

        Args:
            z_tof: ToF高度測定値 [m]
            use_tilt_compensation: 傾斜補正を使用するか
        """
        p_z = self.x[2]
        q = self.x[6:10]

        if use_tilt_compensation:
            # 傾斜補正あり
            R = self.quaternion_to_rotation_matrix(q)
            R_22 = R[2, 2]  # ボディz軸のワールドz軸への射影

            if abs(R_22) < 0.1:
                # 大きく傾いている場合はスキップ
                return

            z_pred = p_z / R_22

            # ヤコビアン
            H = np.zeros((1, 15))
            H[0, 2] = 1.0 / R_22  # ∂h/∂p_z

            # ∂h/∂q = -p_z / R_22^2 * ∂R_22/∂q
            dR22_dq = self._compute_dR22_dq(q)
            H[0, 6:10] = -p_z / (R_22 ** 2) * dR22_dq
        else:
            # 簡易版（小傾斜近似）
            z_pred = p_z
            H = np.zeros((1, 15))
            H[0, 2] = 1.0

        # 観測ノイズ共分散
        R_noise = np.array([[self.params.sigma_tof ** 2]])

        # カルマンゲインと更新
        z_meas = np.array([z_tof])
        z_pred_arr = np.array([z_pred])
        self._kalman_update(z_meas, z_pred_arr, H, R_noise)

    def _compute_dR22_dq(self, q: np.ndarray) -> np.ndarray:
        """
        R[2,2] = 1 - 2*(q_x^2 + q_y^2) のクォータニオンに関する微分

        Returns:
            dR22_dq: 形状(4,)の配列 [∂R22/∂q_w, ∂R22/∂q_x, ∂R22/∂q_y, ∂R22/∂q_z]
        """
        q_x, q_y = q[1], q[2]
        dR22_dq = np.array([0.0, -4.0*q_x, -4.0*q_y, 0.0])
        return dR22_dq

    def update_optical_flow(self, z_flow: np.ndarray, w_meas: np.ndarray):
        """
        オプティカルフローによる更新

        Args:
            z_flow: オプティカルフロー測定値 [flow_x, flow_y] [rad/s]
            w_meas: 角速度測定値 [w_x, w_y, w_z] [rad/s]
        """
        p_z = self.x[2]
        v = self.x[3:6]
        q = self.x[6:10]
        b_g = self.x[10:13]

        # 高度が低すぎる場合はスキップ
        if p_z < 0.1:
            return

        # バイアス補正済み角速度
        w_B = w_meas - b_g

        # ボディ座標系での速度
        R = self.quaternion_to_rotation_matrix(q)
        v_B = R.T @ v

        # 観測予測
        z_pred = np.array([
            (v_B[0] - w_B[1] * p_z) / p_z,
            (v_B[1] + w_B[0] * p_z) / p_z
        ])

        # 観測ヤコビアン H (2x15)
        H = self._compute_optical_flow_jacobian(q, v, p_z, w_B)

        # 観測ノイズ共分散
        R_noise = (self.params.sigma_flow ** 2) * np.eye(2)

        # カルマンゲインと更新
        self._kalman_update(z_flow, z_pred, H, R_noise)

    def _compute_optical_flow_jacobian(self, q: np.ndarray, v: np.ndarray,
                                       p_z: float, w_B: np.ndarray) -> np.ndarray:
        """オプティカルフローの観測ヤコビアン H を計算"""
        H = np.zeros((2, 15))

        R = self.quaternion_to_rotation_matrix(q)
        v_B = R.T @ v

        # ∂h/∂p_z
        H[0, 2] = -(v_B[0] - w_B[1]*p_z) / (p_z ** 2)
        H[1, 2] = -(v_B[1] + w_B[0]*p_z) / (p_z ** 2)

        # ∂h/∂v = (1/p_z) * ∂v_B/∂v = (1/p_z) * R^T
        H[0:2, 3:6] = (1.0 / p_z) * R.T[0:2, :]

        # ∂h/∂q: v_Bのqに関する微分
        # ∂v_B/∂q = ∂(R^T * v)/∂q
        dv_B_dq = self._compute_dRTv_dq(q, v)
        H[0:2, 6:10] = (1.0 / p_z) * dv_B_dq[0:2, :]

        # ∂h/∂b_g: 角速度バイアスの影響
        # flow_x = (v_Bx - (w_y - b_gy)*p_z) / p_z
        # ∂flow_x/∂b_gy = 1
        H[0, 11] = 1.0  # b_gy
        H[1, 10] = -1.0  # b_gx

        return H

    def _compute_dRTv_dq(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """
        R(q)^T * v のクォータニオンに関する微分（R^Tはワールド→ボディ変換）

        Returns:
            dRTv_dq: 3x4行列
        """
        # R^T * v = R(-q) * v と等価（共役クォータニオン）
        # 計算方法は dRa_dq と同様だが、R^T を使用

        q_w, q_x, q_y, q_z = q[0], q[1], q[2], q[3]
        v_x, v_y, v_z = v[0], v[1], v[2]

        # R^T の要素でベクトル変換の微分を計算
        # 簡易実装: 数値微分も可能だが、解析的に計算

        # ∂(R^T*v)/∂q_w
        dRTv_dq_w = 2.0 * np.array([
            v_y*q_z - v_z*q_y,
            v_z*q_x - v_x*q_z,
            v_x*q_y - v_y*q_x
        ])

        # ∂(R^T*v)/∂q_x
        dRTv_dq_x = 2.0 * np.array([
            v_x*q_x + v_y*q_y - 2*v_x*(q_y**2 + q_z**2) + v_z*q_w,
            v_y*q_x - 2*v_y*q_x**2 + v_z*q_z,
            v_z*q_x - v_y*q_w - 2*v_z*q_x**2
        ])

        # より正確な実装は省略（必要に応じて完全実装）
        # 暫定: 簡易的な近似
        dRTv_dq = np.column_stack([dRTv_dq_w, dRTv_dq_x, np.zeros(3), np.zeros(3)])

        return dRTv_dq

    def _kalman_update(self, z_meas: np.ndarray, z_pred: np.ndarray,
                       H: np.ndarray, R_noise: np.ndarray):
        """
        カルマンゲインによる状態・共分散更新

        Args:
            z_meas: 測定値
            z_pred: 観測予測値
            H: 観測ヤコビアン
            R_noise: 観測ノイズ共分散
        """
        # イノベーション（残差）
        y = z_meas - z_pred

        # イノベーション共分散
        S = H @ self.P @ H.T + R_noise

        # 外れ値検出（マハラノビス距離）
        mahalanobis_dist_sq = y.T @ np.linalg.inv(S) @ y
        threshold = 9.21  # 自由度に応じて調整（例: 3-DOFで95%信頼区間）

        if mahalanobis_dist_sq > threshold:
            # 外れ値と判定、更新をスキップ
            print(f"外れ値検出: マハラノビス距離^2 = {mahalanobis_dist_sq:.2f}")
            return

        # カルマンゲイン
        K = self.P @ H.T @ np.linalg.inv(S)

        # 状態更新
        self.x = self.x + K @ y

        # クォータニオンを正規化
        self.x[6:10] = self.normalize_quaternion(self.x[6:10])

        # 共分散更新（Joseph形式で数値安定性向上）
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_noise @ K.T
        self.P = self._make_symmetric(self.P)

    # ========== 状態取得 ==========

    def get_position(self) -> np.ndarray:
        """位置を取得 [m]"""
        return self.x[0:3].copy()

    def get_velocity(self) -> np.ndarray:
        """速度を取得 [m/s]"""
        return self.x[3:6].copy()

    def get_quaternion(self) -> np.ndarray:
        """姿勢クォータニオンを取得"""
        return self.x[6:10].copy()

    def get_euler_angles(self) -> Tuple[float, float, float]:
        """
        オイラー角を取得（ロール、ピッチ、ヨー）[rad]

        Returns:
            (roll, pitch, yaw)
        """
        q = self.x[6:10]
        q_w, q_x, q_y, q_z = q[0], q[1], q[2], q[3]

        # ロール（x軸周り）
        roll = np.arctan2(2*(q_w*q_x + q_y*q_z), 1 - 2*(q_x**2 + q_y**2))

        # ピッチ（y軸周り）
        pitch = np.arcsin(2*(q_w*q_y - q_z*q_x))

        # ヨー（z軸周り）
        yaw = np.arctan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y**2 + q_z**2))

        return roll, pitch, yaw

    def get_gyro_bias(self) -> np.ndarray:
        """ジャイロバイアスを取得 [rad/s]"""
        return self.x[10:13].copy()

    def get_accel_bias(self) -> np.ndarray:
        """加速度バイアスを取得 [m/s²]"""
        return self.x[13:15].copy()

    def get_covariance(self) -> np.ndarray:
        """誤差共分散行列を取得"""
        return self.P.copy()

    def get_position_std(self) -> np.ndarray:
        """位置の標準偏差を取得 [m]"""
        return np.sqrt(np.diag(self.P[0:3, 0:3]))

    def get_attitude_std(self) -> Tuple[float, float, float]:
        """姿勢の標準偏差を取得（近似的にオイラー角） [rad]"""
        q_std = np.sqrt(np.diag(self.P[6:10, 6:10]))
        # クォータニオンからオイラー角の標準偏差への変換は近似
        # 小角度近似: std(roll) ≈ 2*std(q_x), etc.
        return 2*q_std[1], 2*q_std[2], 2*q_std[3]


# ========== 使用例 ==========

if __name__ == "__main__":
    """EKF推定器の使用例"""

    # パラメータ設定
    params = EKFParameters()

    # EKF初期化
    ekf = EKFPositionEstimator(params)

    # サンプリング時間
    dt = 0.01  # 100 Hz

    # シミュレーションループ
    for i in range(1000):
        # IMU測定値（ダミーデータ）
        a_meas = np.array([0.0, 0.0, 9.81])  # 静止時
        w_meas = np.array([0.0, 0.0, 0.0])   # 回転なし

        # 予測ステップ
        ekf.predict(a_meas, w_meas, dt)

        # 更新ステップ（各センサー測定値が利用可能な場合）
        if i % 10 == 0:  # 10 Hzで更新
            # 加速度更新
            ekf.update_accelerometer(a_meas)

            # 地磁気更新（ダミーデータ）
            z_mag = np.array([np.cos(params.mag_inclination), 0.0,
                             np.sin(params.mag_inclination)])
            ekf.update_magnetometer(z_mag)

            # 気圧高度更新
            z_baro = 0.0  # 地上
            ekf.update_barometer(z_baro)

        # 状態取得
        if i % 100 == 0:
            pos = ekf.get_position()
            vel = ekf.get_velocity()
            roll, pitch, yaw = ekf.get_euler_angles()

            print(f"Time: {i*dt:.2f}s")
            print(f"  Position: {pos}")
            print(f"  Velocity: {vel}")
            print(f"  Attitude (deg): roll={np.rad2deg(roll):.1f}, "
                  f"pitch={np.rad2deg(pitch):.1f}, yaw={np.rad2deg(yaw):.1f}")
            print()
