# クアッドコプター位置・姿勢推定EKFアルゴリズム設計書

## 1. 概要

本資料は、クアッドコプターの3次元位置と姿勢をExtended Kalman Filter (EKF)を用いて推定するアルゴリズムの詳細を記述する。

### 1.1 使用センサー

- **IMU（慣性計測装置）**
  - 3軸加速度センサ
  - 3軸角速度センサ（ジャイロ）
- **地磁気センサ（3軸磁気センサ）**
- **オプティカルフローセンサ**
- **ToF（Time-of-Flight）高度センサ**
- **気圧高度センサ**

## 2. 座標系の定義

### 2.1 ワールド座標系（慣性座標系）$\{W\}$

- 原点: 任意の固定点
- $x$軸: 北方向（地磁気北）
- $y$軸: 東方向
- $z$軸: 下方向（NED: North-East-Down座標系）

### 2.2 ボディ座標系（機体座標系）$\{B\}$

- 原点: 機体の重心
- $x$軸: 機体前方
- $y$軸: 機体右方向
- $z$軸: 機体下方向

## 3. 状態ベクトルの定義

状態ベクトル $\mathbf{x}$ は以下の15次元で定義する：

$$
\mathbf{x} = \begin{bmatrix} p_x & p_y & p_z & v_x & v_y & v_z & q_w & q_x & q_y & q_z & b_{gx} & b_{gy} & b_{gz} & b_{ax} & b_{ay} \end{bmatrix}^T
$$

各要素の意味：

| 変数 | 次元 | 説明 |
|------|------|------|
| $\mathbf{p} = [p_x, p_y, p_z]^T$ | 3 | ワールド座標系における位置 [m] |
| $\mathbf{v} = [v_x, v_y, v_z]^T$ | 3 | ワールド座標系における速度 [m/s] |
| $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$ | 4 | 姿勢を表すクォータニオン（正規化制約あり） |
| $\mathbf{b}_g = [b_{gx}, b_{gy}, b_{gz}]^T$ | 3 | ジャイロスコープバイアス [rad/s] |
| $\mathbf{b}_a = [b_{ax}, b_{ay}]^T$ | 2 | 加速度センサ水平バイアス [m/s²] |

**注**: 加速度センサの垂直（$z$軸）バイアスは重力との分離が困難なため、推定しない。

## 4. システムモデル（状態遷移モデル）

### 4.1 連続時間システム

状態の時間微分は以下の運動方程式で表される：

#### 4.1.1 位置の時間微分

$$
\dot{\mathbf{p}} = \mathbf{v}
$$

#### 4.1.2 速度の時間微分

$$
\dot{\mathbf{v}} = R(\mathbf{q}) \mathbf{a}_B + \mathbf{g}_W
$$

ここで：
- $R(\mathbf{q})$: クォータニオン$\mathbf{q}$から計算される回転行列（ボディ座標系からワールド座標系への変換）
- $\mathbf{a}_B = [a_{Bx}, a_{By}, a_{Bz}]^T$: ボディ座標系での加速度（加速度センサ測定値 - バイアス）
- $\mathbf{g}_W = [0, 0, g]^T$: ワールド座標系での重力加速度ベクトル（$g \approx 9.81$ m/s²）

#### 4.1.3 クォータニオンの時間微分

クォータニオンの微分方程式：

$$
\dot{\mathbf{q}} = \frac{1}{2} \Omega(\boldsymbol{\omega}_B) \mathbf{q}
$$

ここで、$\boldsymbol{\omega}_B = [\omega_x, \omega_y, \omega_z]^T$ はボディ座標系での角速度（ジャイロ測定値 - バイアス）。

$\Omega(\boldsymbol{\omega})$ はクォータニオン微分用の$4 \times 4$行列：

$$
\Omega(\boldsymbol{\omega}) = \begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y \\
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

#### 4.1.4 バイアスの時間微分

バイアスはランダムウォークとしてモデル化：

$$
\begin{aligned}
\dot{\mathbf{b}}_g &= \mathbf{w}_{bg} \\
\dot{\mathbf{b}}_a &= \mathbf{w}_{ba}
\end{aligned}
$$

ここで、$\mathbf{w}_{bg}$ と $\mathbf{w}_{ba}$ はプロセスノイズ。

### 4.2 離散化（オイラー法）

サンプリング時間を $\Delta t$ として、離散化システムモデル：

$$
\begin{aligned}
\mathbf{p}[k+1] &= \mathbf{p}[k] + \mathbf{v}[k] \Delta t \\
\mathbf{v}[k+1] &= \mathbf{v}[k] + (R(\mathbf{q}[k]) \mathbf{a}_B[k] + \mathbf{g}_W) \Delta t \\
\mathbf{q}[k+1] &= \mathbf{q}[k] + \frac{1}{2} \Omega(\boldsymbol{\omega}_B[k]) \mathbf{q}[k] \Delta t \\
\mathbf{b}_g[k+1] &= \mathbf{b}_g[k] \\
\mathbf{b}_a[k+1] &= \mathbf{b}_a[k]
\end{aligned}
$$

その後、クォータニオンを正規化：

$$
\mathbf{q}[k+1] = \frac{\mathbf{q}[k+1]}{\|\mathbf{q}[k+1]\|}
$$

### 4.3 クォータニオンから回転行列への変換

クォータニオン $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$ から回転行列 $R$ への変換式：

$$
R(\mathbf{q}) = \begin{bmatrix}
1-2(q_y^2+q_z^2) & 2(q_x q_y - q_w q_z) & 2(q_x q_z + q_w q_y) \\
2(q_x q_y + q_w q_z) & 1-2(q_x^2+q_z^2) & 2(q_y q_z - q_w q_x) \\
2(q_x q_z - q_w q_y) & 2(q_y q_z + q_w q_x) & 1-2(q_x^2+q_y^2)
\end{bmatrix}
$$

## 5. 観測モデル

### 5.1 加速度センサ

ボディ座標系での加速度測定値：

$$
\mathbf{z}_{acc} = R(\mathbf{q})^T (\mathbf{a}_W - \mathbf{g}_W) + \mathbf{b}_{a,3d} + \mathbf{n}_{acc}
$$

ここで：
- $\mathbf{a}_W$: ワールド座標系での加速度 = $\dot{\mathbf{v}}$
- $\mathbf{b}_{a,3d} = [b_{ax}, b_{ay}, 0]^T$: 加速度バイアス（$z$軸はバイアス推定なし）
- $\mathbf{n}_{acc}$: 測定ノイズ $\sim \mathcal{N}(0, R_{acc})$

実用上は、静止または等速飛行時に加速度測定値が重力方向を示すことを利用して姿勢を修正する。

観測方程式（簡易版）：

$$
h_{acc}(\mathbf{x}) = R(\mathbf{q})^T \mathbf{g}_W + \begin{bmatrix} b_{ax} \\ b_{ay} \\ 0 \end{bmatrix}
$$

期待される測定値: $\mathbf{z}_{acc} \approx [0, 0, g]^T$（静止時、ボディ座標系）

### 5.2 角速度センサ（ジャイロ）

ジャイロは直接角速度を測定するため、予測ステップで使用し、通常は観測更新には使用しない。

測定値：

$$
\mathbf{z}_{gyro} = \boldsymbol{\omega}_B + \mathbf{b}_g + \mathbf{n}_{gyro}
$$

### 5.3 地磁気センサ

ボディ座標系での地磁気ベクトル測定：

$$
h_{mag}(\mathbf{x}) = R(\mathbf{q})^T \mathbf{m}_W
$$

ここで：
- $\mathbf{m}_W = [m_N, 0, m_D]^T$: ワールド座標系での地磁気ベクトル（正規化済み）
- $m_N$: 水平北成分
- $m_D$: 垂直下成分（伏角による）

観測方程式：

$$
\mathbf{z}_{mag} = R(\mathbf{q})^T \mathbf{m}_W + \mathbf{n}_{mag}
$$

**実装ポイント**: 地磁気ベクトルは現地の伏角（inclination）と偏角（declination）に依存する。日本では伏角約49度。

### 5.4 気圧高度センサ

高度（下方向を$z$正とするNED座標系）：

$$
h_{baro}(\mathbf{x}) = p_z + n_{baro}
$$

観測方程式：

$$
z_{baro} = p_z
$$

### 5.5 ToF高度センサ

機体下方向の距離測定（地面までの垂直距離）：

$$
h_{tof}(\mathbf{x}) = \frac{p_z}{\cos(\theta)}
$$

ここで、$\theta$は機体の傾斜角。より正確には：

$$
h_{tof}(\mathbf{x}) = \frac{p_z - p_{z,ground}}{R(\mathbf{q})_{[2,2]}}
$$

$R(\mathbf{q})_{[2,2]}$ は回転行列の$(3,3)$成分で、ボディ$z$軸のワールド$z$軸への射影を表す。

**簡易版**（小傾斜角近似）：

$$
h_{tof}(\mathbf{x}) = p_z
$$

### 5.6 オプティカルフロー

オプティカルフローは画像平面上の移動速度を測定する。

ボディ座標系での水平速度との関係：

$$
\begin{aligned}
\text{flow}_x &= \frac{v_{Bx} - \omega_{By} h}{h} \\
\text{flow}_y &= \frac{v_{By} + \omega_{Bx} h}{h}
\end{aligned}
$$

ここで：
- $\mathbf{v}_B = R(\mathbf{q})^T \mathbf{v}$: ボディ座標系での速度
- $h$: 地面までの高度 = $p_z$（またはToF測定値）
- $\omega_{Bx}, \omega_{By}$: ボディ座標系でのロール・ピッチ角速度

観測方程式：

$$
h_{flow}(\mathbf{x}) = \begin{bmatrix}
\frac{v_{Bx} - \omega_{By} p_z}{p_z} \\
\frac{v_{By} + \omega_{Bx} p_z}{p_z}
\end{bmatrix}
$$

測定値：

$$
\mathbf{z}_{flow} = [\text{flow}_x, \text{flow}_y]^T
$$

**注意**: オプティカルフローは高度情報が必要。ToFまたは気圧高度と組み合わせて使用。

## 6. EKFアルゴリズム

### 6.1 初期化

初期状態推定値と初期誤差共分散行列（$15 \times 15$）：

$$
\begin{aligned}
\mathbf{x}[0] &= \mathbf{x}_{init} \\
P[0] &= P_{init}
\end{aligned}
$$

推奨初期値：

$$
\begin{aligned}
\mathbf{p}[0] &= [0, 0, 0]^T \\
\mathbf{v}[0] &= [0, 0, 0]^T \\
\mathbf{q}[0] &= [1, 0, 0, 0]^T \quad \text{(単位クォータニオン)} \\
\mathbf{b}_g[0] &= [0, 0, 0]^T \\
\mathbf{b}_a[0] &= [0, 0]^T
\end{aligned}
$$

初期共分散行列 $P[0]$ は対角行列として：

$$
P[0] = \text{diag}([\sigma_p^2, \sigma_p^2, \sigma_p^2, \sigma_v^2, \sigma_v^2, \sigma_v^2, \sigma_q^2, \sigma_q^2, \sigma_q^2, \sigma_q^2, \sigma_{bg}^2, \sigma_{bg}^2, \sigma_{bg}^2, \sigma_{ba}^2, \sigma_{ba}^2])
$$

推奨値：
- $\sigma_p = 1.0$ [m]
- $\sigma_v = 0.5$ [m/s]
- $\sigma_q = 0.1$ [rad]
- $\sigma_{bg} = 0.01$ [rad/s]
- $\sigma_{ba} = 0.1$ [m/s²]

### 6.2 予測ステップ（Prediction）

#### 6.2.1 状態予測

IMU測定値（加速度 $\mathbf{a}_{meas}$、角速度 $\boldsymbol{\omega}_{meas}$）を用いて：

**バイアス補正：**

$$
\begin{aligned}
\mathbf{a}_B &= \mathbf{a}_{meas} - \begin{bmatrix} b_{ax} \\ b_{ay} \\ 0 \end{bmatrix} \\
\boldsymbol{\omega}_B &= \boldsymbol{\omega}_{meas} - \mathbf{b}_g
\end{aligned}
$$

**状態更新：**

$$
\begin{aligned}
\mathbf{p}_{pred} &= \mathbf{p} + \mathbf{v} \Delta t \\
\mathbf{v}_{pred} &= \mathbf{v} + (R(\mathbf{q}) \mathbf{a}_B + \mathbf{g}_W) \Delta t \\
\mathbf{q}_{pred} &= \mathbf{q} + \frac{1}{2} \Omega(\boldsymbol{\omega}_B) \mathbf{q} \Delta t \\
\mathbf{b}_{g,pred} &= \mathbf{b}_g \\
\mathbf{b}_{a,pred} &= \mathbf{b}_a
\end{aligned}
$$

クォータニオンを正規化：

$$
\mathbf{q}_{pred} = \frac{\mathbf{q}_{pred}}{\|\mathbf{q}_{pred}\|}
$$

予測状態ベクトル：

$$
\mathbf{x}_{pred} = [\mathbf{p}_{pred}; \mathbf{v}_{pred}; \mathbf{q}_{pred}; \mathbf{b}_{g,pred}; \mathbf{b}_{a,pred}]
$$

#### 6.2.2 ヤコビアン行列 $F$ の計算

状態遷移のヤコビアン $F = \frac{\partial f}{\partial \mathbf{x}}$（$15 \times 15$行列）：

$$
F = I + \Delta t \cdot A
$$

ここで、$A$ は連続時間システムのヤコビアン：

$$
A = \begin{bmatrix}
0_{3\times3} & I_{3\times3} & 0_{3\times4} & 0_{3\times3} & 0_{3\times2} \\
0_{3\times3} & 0_{3\times3} & \frac{\partial(R\mathbf{a})}{\partial\mathbf{q}} & R & -R_{[:,:2]} \\
0_{4\times3} & 0_{4\times3} & \frac{\partial\dot{\mathbf{q}}}{\partial\mathbf{q}} & 0_{4\times3} & 0_{4\times2} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times4} & 0_{3\times3} & 0_{3\times2} \\
0_{2\times3} & 0_{2\times3} & 0_{2\times4} & 0_{2\times3} & 0_{2\times2}
\end{bmatrix}
$$

主要な要素：

**速度のクォータニオンに関する微分** $\frac{\partial(R(\mathbf{q})\mathbf{a}_B)}{\partial\mathbf{q}}$：

$$
\begin{aligned}
\frac{\partial(R\mathbf{a})}{\partial q_w} &= 2\begin{bmatrix} a_y q_z - a_z q_y \\ a_z q_x - a_x q_z \\ a_x q_y - a_y q_x \end{bmatrix} \\
\frac{\partial(R\mathbf{a})}{\partial q_x} &= 2\begin{bmatrix} a_y q_y + a_z q_z \\ -a_y q_x + a_z q_w - 2a_x q_x \\ -a_z q_x - a_y q_w - 2a_x q_y \end{bmatrix} \\
\frac{\partial(R\mathbf{a})}{\partial q_y} &= 2\begin{bmatrix} -a_x q_y - a_z q_w - 2a_y q_x \\ a_x q_x + a_z q_z \\ -a_x q_w + a_z q_y - 2a_y q_z \end{bmatrix} \\
\frac{\partial(R\mathbf{a})}{\partial q_z} &= 2\begin{bmatrix} -a_x q_z + a_y q_w - 2a_z q_x \\ -a_y q_z - a_x q_w - 2a_z q_y \\ a_x q_x + a_y q_y \end{bmatrix}
\end{aligned}
$$

**クォータニオンのクォータニオンに関する微分**：

$$
\frac{\partial\dot{\mathbf{q}}}{\partial\mathbf{q}} = \frac{1}{2} \Omega(\boldsymbol{\omega}_B)
$$

**加速度センサバイアスに関する微分**：

速度方程式 $\dot{\mathbf{v}} = R(\mathbf{q}) (\mathbf{a}_{meas} - \mathbf{b}_{a,3d}) + \mathbf{g}$ より、水平バイアス成分のみ：

$$
\frac{\partial\dot{\mathbf{v}}}{\partial\mathbf{b}_a} = -R(\mathbf{q}) \begin{bmatrix} 1 & 0 \\ 0 & 1 \\ 0 & 0 \end{bmatrix}
$$

これは$3 \times 2$行列。

#### 6.2.3 共分散予測

$$
P_{pred} = F P F^T + Q
$$

プロセスノイズ共分散行列 $Q$（$15 \times 15$）は対角行列として：

$$
Q = \Delta t \cdot \text{diag}([0, 0, 0, \sigma_{v,n}^2, \sigma_{v,n}^2, \sigma_{v,n}^2, \sigma_{q,n}^2, \sigma_{q,n}^2, \sigma_{q,n}^2, \sigma_{q,n}^2, \sigma_{bg,n}^2, \sigma_{bg,n}^2, \sigma_{bg,n}^2, \sigma_{ba,n}^2, \sigma_{ba,n}^2])
$$

推奨値（チューニング要）：
- $\sigma_{v,n} = 0.1$ [m/s²]（加速度測定不確かさ）
- $\sigma_{q,n} = 0.001$ [rad/$\sqrt{\text{s}}$]（角速度積分誤差）
- $\sigma_{bg,n} = 0.0001$ [rad/s/$\sqrt{\text{s}}$]（ジャイロバイアスランダムウォーク）
- $\sigma_{ba,n} = 0.001$ [m/s²/$\sqrt{\text{s}}$]（加速度バイアスランダムウォーク）

### 6.3 更新ステップ（Update）

各センサー測定値に対して以下を実行：

#### 6.3.1 観測予測

観測モデル $h(\mathbf{x})$ を用いて：

$$
\mathbf{z}_{pred} = h(\mathbf{x}_{pred})
$$

#### 6.3.2 観測ヤコビアン $H$ の計算

$H = \frac{\partial h}{\partial \mathbf{x}}$（観測次元$\times 15$）

**加速度センサの場合**（$3 \times 15$）：

$$
h_{acc}(\mathbf{x}) = R(\mathbf{q})^T \mathbf{g}_W + \begin{bmatrix} b_{ax} \\ b_{ay} \\ 0 \end{bmatrix}
$$

$$
H_{acc} = \begin{bmatrix} 0_{3\times3} & 0_{3\times3} & \frac{\partial(R^T\mathbf{g})}{\partial\mathbf{q}} & 0_{3\times3} & \begin{bmatrix} 1 & 0 \\ 0 & 1 \\ 0 & 0 \end{bmatrix} \end{bmatrix}
$$

$\frac{\partial(R^T\mathbf{g})}{\partial\mathbf{q}}$ の計算（$\mathbf{g}_W = [0, 0, g]^T$ として）：

$$
\begin{aligned}
\frac{\partial(R^T\mathbf{g})}{\partial q_w} &= 2g\begin{bmatrix} q_y \\ -q_z \\ q_w \end{bmatrix}, \quad
\frac{\partial(R^T\mathbf{g})}{\partial q_x} = 2g\begin{bmatrix} q_z \\ q_w \\ -q_x \end{bmatrix} \\
\frac{\partial(R^T\mathbf{g})}{\partial q_y} &= 2g\begin{bmatrix} q_w \\ q_x \\ -q_y \end{bmatrix}, \quad
\frac{\partial(R^T\mathbf{g})}{\partial q_z} = 2g\begin{bmatrix} -q_x \\ q_y \\ -q_z \end{bmatrix}
\end{aligned}
$$

**地磁気センサの場合**（$3 \times 15$）：

$$
h_{mag}(\mathbf{x}) = R(\mathbf{q})^T \mathbf{m}_W
$$

$$
H_{mag} = \begin{bmatrix} 0_{3\times3} & 0_{3\times3} & \frac{\partial(R^T\mathbf{m})}{\partial\mathbf{q}} & 0_{3\times3} & 0_{3\times2} \end{bmatrix}
$$

$\frac{\partial(R^T\mathbf{m})}{\partial\mathbf{q}}$ の計算は加速度と同様だが、$\mathbf{m}_W$ を使用。

**気圧高度の場合**（$1 \times 15$）：

$$
H_{baro} = \begin{bmatrix} 0 & 0 & 1 & 0_{1\times3} & 0_{1\times4} & 0_{1\times3} & 0_{1\times2} \end{bmatrix}
$$

（$p_z$のみに依存）

**ToF高度の場合**（簡易版、$1 \times 15$）：

$$
H_{tof} = \begin{bmatrix} 0 & 0 & 1 & 0_{1\times3} & 0_{1\times4} & 0_{1\times3} & 0_{1\times2} \end{bmatrix}
$$

**より正確なToF**（傾斜補正あり）：

$$
h_{tof}(\mathbf{x}) = \frac{p_z}{R(\mathbf{q})_{[2,2]}}
$$

$$
H_{tof} = \begin{bmatrix} 0 & 0 & \frac{1}{R_{22}} & 0_{1\times3} & -\frac{p_z}{R_{22}^2} \frac{\partial R_{22}}{\partial\mathbf{q}} & 0_{1\times3} & 0_{1\times2} \end{bmatrix}
$$

$R_{22} = R(\mathbf{q})_{[2,2]} = 1 - 2(q_x^2 + q_y^2)$ より：

$$
\frac{\partial R_{22}}{\partial q_w} = 0, \quad \frac{\partial R_{22}}{\partial q_x} = -4q_x, \quad \frac{\partial R_{22}}{\partial q_y} = -4q_y, \quad \frac{\partial R_{22}}{\partial q_z} = 0
$$

**オプティカルフローの場合**（$2 \times 15$）：

$$
h_{flow}(\mathbf{x}) = \begin{bmatrix}
\frac{v_{Bx} - \omega_{By} p_z}{p_z} \\
\frac{v_{By} + \omega_{Bx} p_z}{p_z}
\end{bmatrix}
$$

ここで、$\mathbf{v}_B = R(\mathbf{q})^T \mathbf{v}$、$\boldsymbol{\omega}_B$ は角速度（ジャイロ測定値）。

位置に関する微分：

$$
\frac{\partial h_{flow}}{\partial p_z} = \begin{bmatrix}
-\frac{v_{Bx} - \omega_{By} p_z}{p_z^2} \\
-\frac{v_{By} + \omega_{Bx} p_z}{p_z^2}
\end{bmatrix}
$$

速度に関する微分：

$$
\frac{\partial h_{flow}}{\partial \mathbf{v}} = \frac{1}{p_z} \begin{bmatrix}
\frac{\partial v_{Bx}}{\partial \mathbf{v}} \\
\frac{\partial v_{By}}{\partial \mathbf{v}}
\end{bmatrix} = \frac{1}{p_z} \begin{bmatrix}
R^T_{[0,:]} \\
R^T_{[1,:]}
\end{bmatrix}
$$

姿勢（クォータニオン）に関する微分：

$$
\frac{\partial h_{flow}}{\partial \mathbf{q}} = \frac{1}{p_z} \begin{bmatrix}
\frac{\partial v_{Bx}}{\partial \mathbf{q}} \\
\frac{\partial v_{By}}{\partial \mathbf{q}}
\end{bmatrix}
$$

$\frac{\partial \mathbf{v}_B}{\partial \mathbf{q}}$ は $\frac{\partial(R^T\mathbf{v})}{\partial\mathbf{q}}$ の計算が必要（加速度の場合と同様の方法）。

完全な $H_{flow}$（$2 \times 15$）：

$$
H_{flow} = \begin{bmatrix} \frac{\partial h}{\partial \mathbf{p}} & \frac{\partial h}{\partial \mathbf{v}} & \frac{\partial h}{\partial \mathbf{q}} & 0_{2\times3} & 0_{2\times2} \end{bmatrix}
$$

#### 6.3.3 イノベーション（残差）

$$
\mathbf{y} = \mathbf{z}_{meas} - \mathbf{z}_{pred}
$$

**注意**: 角度データ（地磁気のヨー角成分など）を含む場合、$-\pi$から$\pi$へのラッピング処理が必要。

#### 6.3.4 イノベーション共分散

$$
S = H P_{pred} H^T + R
$$

観測ノイズ共分散行列 $R$ は各センサーごとに定義：

**加速度センサ**（$3 \times 3$）：

$$
R_{acc} = \text{diag}([\sigma_{acc}^2, \sigma_{acc}^2, \sigma_{acc}^2])
$$

推奨値: $\sigma_{acc} = 0.05 \sim 0.2$ [m/s²]

**地磁気センサ**（$3 \times 3$）：

$$
R_{mag} = \text{diag}([\sigma_{mag}^2, \sigma_{mag}^2, \sigma_{mag}^2])
$$

推奨値: $\sigma_{mag} = 0.1 \sim 0.5$ [正規化単位]

**気圧高度**（$1 \times 1$）：

$$
R_{baro} = \sigma_{baro}^2
$$

推奨値: $\sigma_{baro} = 0.5 \sim 2.0$ [m]

**ToF高度**（$1 \times 1$）：

$$
R_{tof} = \sigma_{tof}^2
$$

推奨値: $\sigma_{tof} = 0.05 \sim 0.2$ [m]

**オプティカルフロー**（$2 \times 2$）：

$$
R_{flow} = \text{diag}([\sigma_{flow}^2, \sigma_{flow}^2])
$$

推奨値: $\sigma_{flow} = 0.5 \sim 2.0$ [rad/s]（ピクセル速度をrad/sに変換後）

#### 6.3.5 カルマンゲイン

$$
K = P_{pred} H^T S^{-1}
$$

#### 6.3.6 状態更新

$$
\mathbf{x}_{updated} = \mathbf{x}_{pred} + K \mathbf{y}
$$

**クォータニオン更新後の正規化**：

$$
\mathbf{q} = \frac{\mathbf{q}}{\|\mathbf{q}\|}
$$

#### 6.3.7 共分散更新

$$
P_{updated} = (I - K H) P_{pred}
$$

または、数値安定性を高めるためにJoseph形式：

$$
P_{updated} = (I - KH) P_{pred} (I - KH)^T + K R K^T
$$

### 6.4 アルゴリズムの流れ

```
初期化: x[0], P[0]

for k = 0, 1, 2, ...
    // 予測ステップ（IMU測定値 a_meas, ω_meas を使用）
    x_pred[k+1] = f(x[k], a_meas[k], ω_meas[k])
    F = ヤコビアン計算
    P_pred[k+1] = F * P[k] * F^T + Q

    // 更新ステップ（利用可能なセンサーごとに実行）
    if 加速度センサ更新タイミング:
        H = H_acc, R = R_acc
        y = z_acc - h_acc(x_pred)
        カルマンゲイン計算・更新

    if 地磁気センサ更新タイミング:
        H = H_mag, R = R_mag
        y = z_mag - h_mag(x_pred)
        カルマンゲイン計算・更新

    if 気圧高度更新タイミング:
        H = H_baro, R = R_baro
        y = z_baro - p_z
        カルマンゲイン計算・更新

    if ToF高度更新タイミング:
        H = H_tof, R = R_tof
        y = z_tof - h_tof(x_pred)
        カルマンゲイン計算・更新

    if オプティカルフロー更新タイミング:
        H = H_flow, R = R_flow
        y = z_flow - h_flow(x_pred)
        カルマンゲイン計算・更新

    x[k+1] = x_updated
    P[k+1] = P_updated
end for
```

## 7. パラメータチューニング

### 7.1 プロセスノイズ共分散 $Q$

$Q$ はシステムモデルの不確かさを表す。大きいほどセンサー測定を信頼する。

**調整方法**：
1. 小さい値から開始
2. 推定値の追従が遅い場合、該当する状態の$Q$を大きくする
3. 推定値が振動する場合、$Q$を小さくする

**推奨初期値**：
- $\sigma_{v,n} = 0.1$ [m/s²]
- $\sigma_{q,n} = 0.001$ [rad/$\sqrt{\text{s}}$]
- $\sigma_{bg,n} = 0.0001$ [rad/s/$\sqrt{\text{s}}$]
- $\sigma_{ba,n} = 0.001$ [m/s²/$\sqrt{\text{s}}$]

### 7.2 観測ノイズ共分散 $R$

$R$ はセンサー測定の不確かさを表す。センサーのデータシートを参考にする。

**調整方法**：
1. データシートの仕様値を初期値とする
2. 静止状態でのセンサー出力の標準偏差を測定
3. 実測値の2~3倍を設定（安全マージン）

**推奨初期値**：

| センサー | パラメータ | 値 |
|---------|-----------|-----|
| 加速度 | $\sigma_{acc}$ | 0.1 m/s² |
| 地磁気 | $\sigma_{mag}$ | 0.3 (正規化) |
| 気圧高度 | $\sigma_{baro}$ | 1.0 m |
| ToF高度 | $\sigma_{tof}$ | 0.1 m |
| オプティカルフロー | $\sigma_{flow}$ | 1.0 rad/s |

### 7.3 初期共分散 $P[0]$

初期状態の不確かさを表す。

**推奨値**：
- $\sigma_p = 1.0$ [m]
- $\sigma_v = 0.5$ [m/s]
- $\sigma_q = 0.1$ [rad]
- $\sigma_{bg} = 0.01$ [rad/s]
- $\sigma_{ba} = 0.1$ [m/s²]

### 7.4 チューニング手順

1. **静止状態テスト**：
   - 機体を静止させ、推定位置・姿勢が安定することを確認
   - 加速度センサと地磁気センサでロール・ピッチ・ヨーが正しく推定されるか確認

2. **高度テスト**：
   - 機体を上下させ、気圧高度とToF高度の融合を確認
   - ToFは近距離で精度が高いことを確認

3. **水平移動テスト**：
   - オプティカルフローで水平速度が推定されるか確認
   - 高度情報との組み合わせで位置が推定されるか確認

4. **回転テスト**：
   - 各軸周りに回転させ、姿勢推定の精度を確認
   - ジャイロバイアスが適切に推定されるか確認

5. **動的飛行テスト**：
   - 実際の飛行パターンで全センサーが統合されることを確認
   - 推定値の発散や遅延がないか確認

### 7.5 アダプティブチューニング（発展）

実時間でパラメータを調整する方法：

**イノベーションベース調整：**

イノベーション $\mathbf{y}$ の大きさに基づいて動的に調整：

$$
d^2 = \mathbf{y}^T S^{-1} \mathbf{y}
$$

- **外れ値対策**: $d^2$ が $S$ の対角成分より大きい場合 → $R$ を一時的に大きくする
- **過剰補正防止**: $\mathbf{y}$ が継続的に小さい場合 → $Q$ を小さくする、または $R$ を大きくする

## 8. 実装上の注意点

### 8.1 クォータニオン正規化

毎ステップ後に必ず正規化：

$$
\mathbf{q} = \frac{\mathbf{q}}{\sqrt{q_w^2 + q_x^2 + q_y^2 + q_z^2}}
$$

### 8.2 共分散行列の対称性維持

数値誤差により $P$ が非対称になる場合：

$$
P = \frac{P + P^T}{2}
$$

### 8.3 共分散行列の正定値性

固有値が負にならないよう、定期的にチェック。対策：
- Joseph形式の更新式を使用
- 最小固有値に下限を設定（例: $10^{-9}$）

### 8.4 センサー更新頻度

各センサーの更新レートが異なる場合の処理：

```
IMU（加速度・ジャイロ）: 100 ~ 1000 Hz → 予測ステップに使用
地磁気: 10 ~ 100 Hz → 低頻度更新
気圧: 10 ~ 50 Hz
ToF: 10 ~ 100 Hz
オプティカルフロー: 10 ~ 100 Hz
```

予測ステップはIMUレートで実行し、他センサーは到着時に更新ステップを実行。

### 8.5 外れ値検出

イノベーションのマハラノビス距離でチェック：

$$
d^2 = \mathbf{y}^T S^{-1} \mathbf{y}
$$

判定基準：

- $d^2 > \chi^2_{\text{threshold}}$ の場合、この測定値を棄却
- 例: 3自由度で95%信頼区間の場合、$\chi^2_{\text{threshold}} = 9.21$

### 8.6 地磁気の磁気擾乱対策

室内や金属近くでは地磁気が乱れる。対策：
- 地磁気更新を一時停止（磁場強度が閾値外の場合）
- ジャイロのみでヨー角を推定（短期間）

### 8.7 オプティカルフローの高度依存性

高度が0に近い場合、フロー計算で除算エラーが発生する。

最小高度閾値：

$$
h_{min} = 0.1 \text{ m}
$$

対策: $p_z < h_{min}$ の場合、オプティカルフロー更新をスキップ

## 9. 実装チェックリスト

- [ ] 状態ベクトルの初期化
- [ ] クォータニオンから回転行列への変換関数
- [ ] 予測ステップの実装
- [ ] 各センサーの観測モデル実装
- [ ] ヤコビアン行列$F$, $H$の計算
- [ ] カルマンゲイン計算
- [ ] 更新ステップの実装
- [ ] クォータニオン正規化
- [ ] パラメータ$Q$, $R$の設定
- [ ] センサー更新頻度の管理
- [ ] 外れ値検出機構
- [ ] ログ・デバッグ機能

## 10. 拡張・改善案

### 10.1 GNSS（GPS）の追加

位置の絶対値を観測：

$$
h_{gps}(\mathbf{x}) = \begin{bmatrix} p_x \\ p_y \\ p_z \end{bmatrix}
$$

観測ノイズ: $\sigma_{gps} = 2 \sim 10$ m（環境依存）

### 10.2 UWB測距システム

複数のアンカーからの距離測定で位置を推定。

### 10.3 Unscented Kalman Filter (UKF)

非線形性が強い場合、UKFが有効。ヤコビアン計算不要。

### 10.4 センサーフュージョンアーキテクチャ

- **疎結合**: 各センサーで独立推定後に統合
- **密結合**: 本資料のように生データを直接EKFで統合（推奨）

## 11. 参考文献・リソース

1. **Quaternion kinematics for the error-state Kalman filter** (Joan Solà, 2017)
   - クォータニオンベースEKFの詳細

2. **Optical Flow and Vision-based Navigation**
   - オプティカルフローの詳細な数学的扱い

3. **Barometer and GPS fusion**
   - 高度推定の実践的手法

## 12. コード実装例の準備

本資料の数式を直接実装可能なPythonコードのスケルトンを別ファイル [`ekf_position_estimator.py`](ekf_position_estimator.py) で提供する。

主要な関数：
- `quaternion_to_rotation_matrix(q)`: クォータニオン→回転行列
- `predict_step(x, P, a_meas, w_meas, dt, Q)`: 予測ステップ
- `update_step_accelerometer(x, P, z_acc, R_acc)`: 加速度更新
- `update_step_magnetometer(x, P, z_mag, R_mag, m_W)`: 地磁気更新
- `update_step_barometer(x, P, z_baro, R_baro)`: 気圧更新
- `update_step_tof(x, P, z_tof, R_tof)`: ToF更新
- `update_step_optical_flow(x, P, z_flow, w_meas, R_flow)`: フロー更新

---

**本資料は実装可能な数式レベルでEKFアルゴリズムを記述している。次のステップとして、具体的なPython/C++コードへの実装を推奨する。**
