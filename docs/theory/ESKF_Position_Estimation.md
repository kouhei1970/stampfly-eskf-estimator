# クアッドコプター位置・姿勢推定 ESKF完全設計書

## 1. 概要

本資料は、クアッドコプターの3次元位置と姿勢を**Error-State Kalman Filter (ESKF)** を用いて推定するアルゴリズムの完全な設計書である。ESKFは標準EKFよりも数値安定性が高く、姿勢推定において広く使われている手法である。

### 1.1 ESKFの利点

**標準EKF vs ESKF:**

| 特徴 | 標準EKF | ESKF |
|------|---------|------|
| 推定対象 | 状態そのもの | エラー状態（小さな値） |
| 数値安定性 | 中程度 | **高い** |
| クォータニオン制約 | 毎回正規化が必要 | **自然に保たれる** |
| 線形化精度 | 大きな値での線形化 | **小角度近似が使える** |
| 計算効率 | 中程度 | **高い** |

**ESKFが優れている理由:**

1. **小さな誤差を推定**: エラー状態（小さな値）を推定するため、線形化誤差が小さい
2. **姿勢の扱いが簡潔**: 姿勢誤差を回転ベクトル（3次元）で表現できる
3. **数値安定**: 共分散行列が小さな値を扱うため、数値的に安定
4. **計算効率**: エラー状態の次元が小さい（姿勢が4次元→3次元）

### 1.2 使用センサー

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
- $z$軸: 下方向（**NED: North-East-Down座標系**）

### 2.2 ボディ座標系（機体座標系）$\{B\}$

- 原点: 機体の重心
- $x$軸: 機体前方
- $y$軸: 機体右方向
- $z$軸: 機体下方向

## 3. ESKFの基本概念

### 3.1 状態の分解

ESKFでは、真の状態を**ノミナル状態**と**エラー状態**に分解する：

$$
\mathbf{x}_{\text{true}} = \mathbf{x}_{\text{nominal}} \oplus \delta\mathbf{x}_{\text{error}}
$$

ここで：
- $\mathbf{x}_{\text{nominal}}$: ノミナル状態（センサー測定から推定される大まかな状態）
- $\delta\mathbf{x}_{\text{error}}$: エラー状態（ノミナル状態からのずれ、小さな値）
- $\oplus$: 合成演算子（状態の種類によって異なる）

### 3.2 ノミナル状態ベクトル

ノミナル状態 $\mathbf{x}$ は以下の15次元で定義：

$$
\mathbf{x} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \mathbf{q} \\ \mathbf{b}_g \\ \mathbf{b}_a \end{bmatrix} = \begin{bmatrix} p_x \\ p_y \\ p_z \\ v_x \\ v_y \\ v_z \\ q_w \\ q_x \\ q_y \\ q_z \\ b_{gx} \\ b_{gy} \\ b_{gz} \\ b_{ax} \\ b_{ay} \end{bmatrix}
$$

各要素の意味：

| 変数 | 次元 | 説明 |
|------|------|------|
| $\mathbf{p} = [p_x, p_y, p_z]^T$ | 3 | ワールド座標系における位置 [m] |
| $\mathbf{v} = [v_x, v_y, v_z]^T$ | 3 | ワールド座標系における速度 [m/s] |
| $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$ | 4 | 姿勢を表すクォータニオン |
| $\mathbf{b}_g = [b_{gx}, b_{gy}, b_{gz}]^T$ | 3 | ジャイロスコープバイアス [rad/s] |
| $\mathbf{b}_a = [b_{ax}, b_{ay}]^T$ | 2 | 加速度センサ水平バイアス [m/s²] |

### 3.3 エラー状態ベクトル

**重要**: エラー状態は**18次元ではなく15次元**で定義される（姿勢誤差を3次元の回転ベクトルで表現）：

$$
\delta\mathbf{x} = \begin{bmatrix} \delta\mathbf{p} \\ \delta\mathbf{v} \\ \delta\boldsymbol{\theta} \\ \delta\mathbf{b}_g \\ \delta\mathbf{b}_a \end{bmatrix}
$$

各要素の意味：

| 変数 | 次元 | 説明 | 合成方法 |
|------|------|------|----------|
| $\delta\mathbf{p}$ | 3 | 位置誤差 [m] | 加算: $\mathbf{p}_{\text{true}} = \mathbf{p} + \delta\mathbf{p}$ |
| $\delta\mathbf{v}$ | 3 | 速度誤差 [m/s] | 加算: $\mathbf{v}_{\text{true}} = \mathbf{v} + \delta\mathbf{v}$ |
| $\delta\boldsymbol{\theta}$ | 3 | **姿勢誤差（回転ベクトル）** [rad] | 回転合成: $\mathbf{q}_{\text{true}} = \mathbf{q} \otimes \delta\mathbf{q}(\delta\boldsymbol{\theta})$ |
| $\delta\mathbf{b}_g$ | 3 | ジャイロバイアス誤差 [rad/s] | 加算: $\mathbf{b}_{g,\text{true}} = \mathbf{b}_g + \delta\mathbf{b}_g$ |
| $\delta\mathbf{b}_a$ | 2 | 加速度バイアス誤差 [m/s²] | 加算: $\mathbf{b}_{a,\text{true}} = \mathbf{b}_a + \delta\mathbf{b}_a$ |

**姿勢誤差の扱い（重要）:**

姿勢誤差 $\delta\boldsymbol{\theta} = [\delta\theta_x, \delta\theta_y, \delta\theta_z]^T$ は小角度回転ベクトルで、クォータニオンに変換される：

$$
\delta\mathbf{q}(\delta\boldsymbol{\theta}) \approx \begin{bmatrix} 1 \\ \frac{1}{2}\delta\theta_x \\ \frac{1}{2}\delta\theta_y \\ \frac{1}{2}\delta\theta_z \end{bmatrix} \quad \text{(小角度近似)}
$$

真のクォータニオン：

$$
\mathbf{q}_{\text{true}} = \mathbf{q} \otimes \delta\mathbf{q}(\delta\boldsymbol{\theta})
$$

ここで $\otimes$ はクォータニオン乗算。

## 4. システムモデル

### 4.1 ノミナル状態の連続時間伝播

ノミナル状態は、センサー測定値を用いて**非線形運動方程式**で伝播される：

#### 4.1.1 位置の時間微分

$$
\dot{\mathbf{p}} = \mathbf{v}
$$

#### 4.1.2 速度の時間微分

$$
\dot{\mathbf{v}} = R(\mathbf{q}) (\mathbf{a}_m - \mathbf{b}_a^{3d}) + \mathbf{g}_W
$$

ここで：
- $R(\mathbf{q})$: クォータニオン$\mathbf{q}$から計算される回転行列（ボディ→ワールド）
- $\mathbf{a}_m = [a_{mx}, a_{my}, a_{mz}]^T$: 加速度センサ測定値 [m/s²]
- $\mathbf{b}_a^{3d} = [b_{ax}, b_{ay}, 0]^T$: 加速度バイアス（z軸は推定しない）
- $\mathbf{g}_W = [0, 0, g]^T$: 重力加速度 ($g \approx 9.81$ m/s²)

#### 4.1.3 クォータニオンの時間微分

$$
\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \boldsymbol{\omega}_m^q - \frac{1}{2} \mathbf{q} \otimes \mathbf{b}_g^q
$$

または行列形式：

$$
\dot{\mathbf{q}} = \frac{1}{2} \Omega(\boldsymbol{\omega}_m - \mathbf{b}_g) \mathbf{q}
$$

ここで：
- $\boldsymbol{\omega}_m = [\omega_{mx}, \omega_{my}, \omega_{mz}]^T$: 角速度センサ測定値 [rad/s]
- $\Omega(\boldsymbol{\omega})$: クォータニオン微分用の$4 \times 4$行列

$$
\Omega(\boldsymbol{\omega}) = \begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y \\
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

#### 4.1.4 バイアスの時間微分

バイアスはランダムウォーク：

$$
\begin{aligned}
\dot{\mathbf{b}}_g &= \mathbf{0} \quad \text{(プロセスノイズで変動)} \\
\dot{\mathbf{b}}_a &= \mathbf{0} \quad \text{(プロセスノイズで変動)}
\end{aligned}
$$

### 4.2 エラー状態の連続時間伝播

エラー状態は**線形化された運動方程式**で伝播される：

$$
\delta\dot{\mathbf{x}} = F_c \delta\mathbf{x} + G_c \mathbf{n}
$$

ここで：
- $F_c$: 連続時間システム行列（15×15）
- $G_c$: ノイズ入力行列
- $\mathbf{n}$: プロセスノイズベクトル

#### 4.2.1 エラー状態の微分方程式

位置誤差：

$$
\delta\dot{\mathbf{p}} = \delta\mathbf{v}
$$

速度誤差：

$$
\delta\dot{\mathbf{v}} = -R(\mathbf{q}) [\mathbf{a}_m - \mathbf{b}_a^{3d}]_\times \delta\boldsymbol{\theta} - R(\mathbf{q}) \delta\mathbf{b}_a^{3d} + \mathbf{n}_a
$$

ここで $[\mathbf{a}]_\times$ は歪対称行列（skew-symmetric matrix）：

$$
[\mathbf{a}]_\times = \begin{bmatrix}
0 & -a_z & a_y \\
a_z & 0 & -a_x \\
-a_y & a_x & 0
\end{bmatrix}
$$

姿勢誤差（回転ベクトル）：

$$
\delta\dot{\boldsymbol{\theta}} = -[\boldsymbol{\omega}_m - \mathbf{b}_g]_\times \delta\boldsymbol{\theta} - \delta\mathbf{b}_g + \mathbf{n}_\omega
$$

バイアス誤差：

$$
\begin{aligned}
\delta\dot{\mathbf{b}}_g &= \mathbf{n}_{bg} \\
\delta\dot{\mathbf{b}}_a &= \mathbf{n}_{ba}
\end{aligned}
$$

#### 4.2.2 連続時間システム行列 $F_c$

$$
F_c = \begin{bmatrix}
0_{3\times3} & I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times2} \\
0_{3\times3} & 0_{3\times3} & -R(\mathbf{q})[\mathbf{a}_m - \mathbf{b}_a^{3d}]_\times & 0_{3\times3} & -R(\mathbf{q})_{[:,:2]} \\
0_{3\times3} & 0_{3\times3} & -[\boldsymbol{\omega}_m - \mathbf{b}_g]_\times & -I_{3\times3} & 0_{3\times2} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times2} \\
0_{2\times3} & 0_{2\times3} & 0_{2\times3} & 0_{2\times3} & 0_{2\times2}
\end{bmatrix}
$$

ここで $R(\mathbf{q})_{[:,:2]}$ は回転行列の最初の2列。

### 4.3 離散化

サンプリング時間 $\Delta t$ で離散化：

**ノミナル状態の離散化（オイラー法）:**

$$
\begin{aligned}
\mathbf{p}[k+1] &= \mathbf{p}[k] + \mathbf{v}[k] \Delta t \\
\mathbf{v}[k+1] &= \mathbf{v}[k] + \left( R(\mathbf{q}[k]) (\mathbf{a}_m[k] - \mathbf{b}_a^{3d}[k]) + \mathbf{g}_W \right) \Delta t \\
\mathbf{q}[k+1] &= \mathbf{q}[k] + \frac{1}{2} \Omega(\boldsymbol{\omega}_m[k] - \mathbf{b}_g[k]) \mathbf{q}[k] \Delta t \\
\mathbf{q}[k+1] &= \frac{\mathbf{q}[k+1]}{\|\mathbf{q}[k+1]\|} \quad \text{(正規化)} \\
\mathbf{b}_g[k+1] &= \mathbf{b}_g[k] \\
\mathbf{b}_a[k+1] &= \mathbf{b}_a[k]
\end{aligned}
$$

**エラー状態の離散化:**

離散時間システム行列：

$$
F_d = I + F_c \Delta t + \frac{1}{2} F_c^2 \Delta t^2 + \cdots \approx I + F_c \Delta t
$$

（1次近似で十分精度が高い）

エラー状態共分散の伝播：

$$
P[k+1] = F_d P[k] F_d^T + Q_d
$$

プロセスノイズ共分散 $Q_d$：

$$
Q_d = \Delta t \cdot \text{diag}([0, 0, 0, \sigma_{a,n}^2, \sigma_{a,n}^2, \sigma_{a,n}^2, \sigma_{\omega,n}^2, \sigma_{\omega,n}^2, \sigma_{\omega,n}^2, \sigma_{bg,n}^2, \sigma_{bg,n}^2, \sigma_{bg,n}^2, \sigma_{ba,n}^2, \sigma_{ba,n}^2])
$$

推奨値：
- $\sigma_{a,n} = 0.1$ [m/s²]（加速度ノイズ）
- $\sigma_{\omega,n} = 0.001$ [rad/s]（角速度ノイズ）
- $\sigma_{bg,n} = 0.0001$ [rad/s/$\sqrt{\text{s}}$]（ジャイロバイアスドリフト）
- $\sigma_{ba,n} = 0.001$ [m/s²/$\sqrt{\text{s}}$]（加速度バイアスドリフト）

### 4.4 クォータニオンから回転行列への変換

$$
R(\mathbf{q}) = \begin{bmatrix}
1-2(q_y^2+q_z^2) & 2(q_x q_y - q_w q_z) & 2(q_x q_z + q_w q_y) \\
2(q_x q_y + q_w q_z) & 1-2(q_x^2+q_z^2) & 2(q_y q_z - q_w q_x) \\
2(q_x q_z - q_w q_y) & 2(q_y q_z + q_w q_x) & 1-2(q_x^2+q_y^2)
\end{bmatrix}
$$

## 5. 観測モデル

観測は**ノミナル状態に対して**行われる。エラー状態は観測を通じて修正される。

### 5.1 加速度センサ

**ノミナル観測モデル:**

$$
\mathbf{z}_{acc} = R(\mathbf{q})^T \mathbf{g}_W + \mathbf{b}_a^{3d} + \mathbf{n}_{acc}
$$

期待される測定値（静止時）：$\mathbf{z}_{acc} \approx [0, 0, g]^T$

**観測ヤコビアン** $H_{acc}$（3×15）:

$$
H_{acc} = \begin{bmatrix} 0_{3\times3} & 0_{3\times3} & -R(\mathbf{q})^T [\mathbf{g}_W]_\times & 0_{3\times3} & I_{3\times2} \end{bmatrix}
$$

ここで $I_{3\times2} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \\ 0 & 0 \end{bmatrix}$

**観測ノイズ共分散:**

$$
R_{acc} = \sigma_{acc}^2 I_{3\times3}, \quad \sigma_{acc} = 0.05 \sim 0.2 \text{ [m/s²]}
$$

### 5.2 地磁気センサ

**ノミナル観測モデル:**

$$
\mathbf{z}_{mag} = R(\mathbf{q})^T \mathbf{m}_W + \mathbf{n}_{mag}
$$

ここで $\mathbf{m}_W = [m_N, 0, m_D]^T$ は正規化された地磁気ベクトル（ワールド座標系）。

**観測ヤコビアン** $H_{mag}$（3×15）:

$$
H_{mag} = \begin{bmatrix} 0_{3\times3} & 0_{3\times3} & -R(\mathbf{q})^T [\mathbf{m}_W]_\times & 0_{3\times3} & 0_{3\times2} \end{bmatrix}
$$

**観測ノイズ共分散:**

$$
R_{mag} = \sigma_{mag}^2 I_{3\times3}, \quad \sigma_{mag} = 0.1 \sim 0.5
$$

### 5.3 気圧高度センサ

**ノミナル観測モデル:**

$$
z_{baro} = p_z + n_{baro}
$$

**観測ヤコビアン** $H_{baro}$（1×15）:

$$
H_{baro} = \begin{bmatrix} 0 & 0 & 1 & 0_{1\times12} \end{bmatrix}
$$

**観測ノイズ:**

$$
R_{baro} = \sigma_{baro}^2, \quad \sigma_{baro} = 0.5 \sim 2.0 \text{ [m]}
$$

### 5.4 ToF高度センサ

**簡易版（小傾斜近似）:**

$$
z_{tof} = p_z + n_{tof}
$$

**観測ヤコビアン:**

$$
H_{tof} = \begin{bmatrix} 0 & 0 & 1 & 0_{1\times12} \end{bmatrix}
$$

**観測ノイズ:**

$$
R_{tof} = \sigma_{tof}^2, \quad \sigma_{tof} = 0.05 \sim 0.2 \text{ [m]}
$$

**傾斜補正版:**

$$
z_{tof} = \frac{p_z}{R(\mathbf{q})_{[2,2]}} + n_{tof}
$$

### 5.5 オプティカルフロー

**ノミナル観測モデル:**

$$
\mathbf{z}_{flow} = \begin{bmatrix}
\frac{v_{Bx} - \omega_{By} p_z}{p_z} \\
\frac{v_{By} + \omega_{Bx} p_z}{p_z}
\end{bmatrix} + \mathbf{n}_{flow}
$$

ここで $\mathbf{v}_B = R(\mathbf{q})^T \mathbf{v}$ はボディ座標系での速度。

**観測ヤコビアン** $H_{flow}$（2×15）:

$$
H_{flow} = \begin{bmatrix}
\frac{\partial h_{flow}}{\partial \mathbf{p}} & \frac{\partial h_{flow}}{\partial \mathbf{v}} & \frac{\partial h_{flow}}{\partial \delta\boldsymbol{\theta}} & 0_{2\times3} & 0_{2\times2}
\end{bmatrix}
$$

詳細な微分計算は実装セクションで提供。

**観測ノイズ:**

$$
R_{flow} = \sigma_{flow}^2 I_{2\times2}, \quad \sigma_{flow} = 0.5 \sim 2.0 \text{ [rad/s]}
$$

## 6. ESKFアルゴリズムの完全フロー

### 6.1 初期化

**ノミナル状態:**

$$
\mathbf{x}[0] = \begin{bmatrix} \mathbf{0}_3 \\ \mathbf{0}_3 \\ [1, 0, 0, 0]^T \\ \mathbf{0}_3 \\ \mathbf{0}_2 \end{bmatrix}
$$

**エラー状態:**

$$
\delta\mathbf{x}[0] = \mathbf{0}_{15}
$$

**エラー共分散:**

$$
P[0] = \text{diag}([\sigma_p^2 \cdot \mathbf{1}_3, \sigma_v^2 \cdot \mathbf{1}_3, \sigma_\theta^2 \cdot \mathbf{1}_3, \sigma_{bg}^2 \cdot \mathbf{1}_3, \sigma_{ba}^2 \cdot \mathbf{1}_2])
$$

推奨値：
- $\sigma_p = 1.0$ [m]
- $\sigma_v = 0.5$ [m/s]
- $\sigma_\theta = 0.1$ [rad]
- $\sigma_{bg} = 0.01$ [rad/s]
- $\sigma_{ba} = 0.1$ [m/s²]

### 6.2 予測ステップ

#### ステップ1: ノミナル状態の伝播

IMU測定値 $\mathbf{a}_m, \boldsymbol{\omega}_m$ を使用：

$$
\begin{aligned}
\mathbf{p}[k+1] &= \mathbf{p}[k] + \mathbf{v}[k] \Delta t \\
\mathbf{v}[k+1] &= \mathbf{v}[k] + \left( R(\mathbf{q}[k]) (\mathbf{a}_m[k] - \mathbf{b}_a^{3d}[k]) + \mathbf{g}_W \right) \Delta t \\
\mathbf{q}[k+1] &= \text{normalize}\left( \mathbf{q}[k] + \frac{1}{2} \Omega(\boldsymbol{\omega}_m[k] - \mathbf{b}_g[k]) \mathbf{q}[k] \Delta t \right) \\
\mathbf{b}_g[k+1] &= \mathbf{b}_g[k] \\
\mathbf{b}_a[k+1] &= \mathbf{b}_a[k]
\end{aligned}
$$

#### ステップ2: エラー状態共分散の伝播

システム行列 $F_d = I + F_c \Delta t$ を計算：

$$
P[k+1] = F_d P[k] F_d^T + Q_d
$$

対称化：

$$
P[k+1] = \frac{P[k+1] + P[k+1]^T}{2}
$$

**重要**: エラー状態 $\delta\mathbf{x}$ 自体は予測で0のまま（リセット後は常に0）。

### 6.3 更新ステップ

各センサー測定値が到着したら：

#### ステップ1: イノベーション計算

$$
\mathbf{y} = \mathbf{z}_{meas} - h(\mathbf{x})
$$

ここで $h(\mathbf{x})$ はノミナル状態に対する観測予測。

#### ステップ2: イノベーション共分散

$$
S = H P H^T + R_{sensor}
$$

#### ステップ3: カルマンゲイン

$$
K = P H^T S^{-1}
$$

#### ステップ4: エラー状態の更新

$$
\delta\mathbf{x} = K \mathbf{y}
$$

#### ステップ5: エラー共分散の更新

Joseph形式（数値安定）：

$$
P = (I - KH) P (I - KH)^T + K R_{sensor} K^T
$$

#### ステップ6: ノミナル状態へのエラー状態の注入（リセット）

**位置・速度・バイアス（加算）:**

$$
\begin{aligned}
\mathbf{p} &\leftarrow \mathbf{p} + \delta\mathbf{p} \\
\mathbf{v} &\leftarrow \mathbf{v} + \delta\mathbf{v} \\
\mathbf{b}_g &\leftarrow \mathbf{b}_g + \delta\mathbf{b}_g \\
\mathbf{b}_a &\leftarrow \mathbf{b}_a + \delta\mathbf{b}_a
\end{aligned}
$$

**姿勢（クォータニオン乗算）:**

回転ベクトル $\delta\boldsymbol{\theta}$ をクォータニオンに変換：

$$
\delta\mathbf{q} = \begin{bmatrix} \cos(\frac{\|\delta\boldsymbol{\theta}\|}{2}) \\ \frac{\delta\boldsymbol{\theta}}{\|\delta\boldsymbol{\theta}\|} \sin(\frac{\|\delta\boldsymbol{\theta}\|}{2}) \end{bmatrix}
$$

小角度近似（$\|\delta\boldsymbol{\theta}\| \ll 1$）：

$$
\delta\mathbf{q} \approx \begin{bmatrix} 1 \\ \frac{1}{2}\delta\boldsymbol{\theta} \end{bmatrix} = \begin{bmatrix} 1 \\ \frac{1}{2}\delta\theta_x \\ \frac{1}{2}\delta\theta_y \\ \frac{1}{2}\delta\theta_z \end{bmatrix}
$$

ノミナル姿勢を更新：

$$
\mathbf{q} \leftarrow \mathbf{q} \otimes \delta\mathbf{q}
$$

正規化：

$$
\mathbf{q} \leftarrow \frac{\mathbf{q}}{\|\mathbf{q}\|}
$$

#### ステップ7: エラー状態のリセット

$$
\delta\mathbf{x} \leftarrow \mathbf{0}_{15}
$$

**重要**: エラー状態は毎回リセットされるため、常に小さな値のまま。これがESKFの核心。

#### ステップ8: エラー共分散のリセット（オプション）

姿勢のエラーが大きい場合、共分散行列も補正が必要な場合がある（詳細は発展的内容）。

### 6.4 アルゴリズム全体のフロー

```
初期化: x[0], δx[0]=0, P[0]

for k = 0, 1, 2, ...
    // 予測ステップ
    ノミナル状態 x を IMU測定値で伝播
    エラー共分散 P を F_d, Q_d で伝播
    (δx は 0 のまま)

    // 更新ステップ（センサー測定値が到着したら）
    for each センサー測定 z:
        イノベーション y = z - h(x)
        S = H*P*H^T + R
        K = P*H^T*S^(-1)

        エラー状態更新: δx = K*y
        エラー共分散更新: P = (I-KH)*P*(I-KH)^T + K*R*K^T

        // エラー状態をノミナル状態に注入
        x ← x ⊕ δx  (加算または回転合成)

        // エラー状態リセット
        δx ← 0

    end for
end for
```

## 7. パラメータチューニング

### 7.1 プロセスノイズ $Q_d$

| パラメータ | 推奨値 | 説明 |
|-----------|--------|------|
| $\sigma_{a,n}$ | 0.1 m/s² | 加速度ノイズ |
| $\sigma_{\omega,n}$ | 0.001 rad/s | 角速度ノイズ |
| $\sigma_{bg,n}$ | 0.0001 rad/s/√s | ジャイロバイアスドリフト |
| $\sigma_{ba,n}$ | 0.001 m/s²/√s | 加速度バイアスドリフト |

### 7.2 観測ノイズ $R$

| センサー | パラメータ | 推奨値 |
|---------|-----------|--------|
| 加速度 | $\sigma_{acc}$ | 0.05~0.2 m/s² |
| 地磁気 | $\sigma_{mag}$ | 0.1~0.5 |
| 気圧高度 | $\sigma_{baro}$ | 0.5~2.0 m |
| ToF高度 | $\sigma_{tof}$ | 0.05~0.2 m |
| オプティカルフロー | $\sigma_{flow}$ | 0.5~2.0 rad/s |

### 7.3 チューニング手順

1. **静止状態テスト**: 位置・姿勢が安定することを確認
2. **高度テスト**: 気圧・ToF融合を確認
3. **水平移動テスト**: オプティカルフローとの統合を確認
4. **回転テスト**: 姿勢推定精度を確認
5. **動的飛行テスト**: 全センサー統合を確認

## 8. 実装上の重要ポイント

### 8.1 エラー状態は常に0付近

ESKFの核心：
- エラー状態 $\delta\mathbf{x}$ は更新後すぐにリセットされる
- 常に小さな値を扱うため、線形化誤差が小さい
- 共分散 $P$ も小さな値を扱うため数値安定

### 8.2 姿勢誤差の扱い

- 姿勢誤差は3次元の回転ベクトル $\delta\boldsymbol{\theta}$ で表現
- クォータニオン4次元より1次元少ない→計算効率向上
- 小角度近似が使える

### 8.3 外れ値検出

マハラノビス距離：

$$
d^2 = \mathbf{y}^T S^{-1} \mathbf{y}
$$

$d^2 > \chi^2_{\text{threshold}}$ なら測定値を棄却。

### 8.4 数値安定性

- Joseph形式の共分散更新を使用
- 共分散の対称化: $P = \frac{P + P^T}{2}$
- クォータニオンの正規化を毎ステップ実行

## 9. ESKFと標準EKFの比較まとめ

| 項目 | 標準EKF | ESKF |
|------|---------|------|
| 推定対象 | 状態 $\mathbf{x}$ | エラー状態 $\delta\mathbf{x}$ |
| エラー状態次元 | - | 15次元（姿勢が3次元） |
| ノミナル状態次元 | 15次元 | 15次元（姿勢が4次元） |
| 姿勢表現 | クォータニオン4次元 | エラー：回転ベクトル3次元 |
| 線形化 | 大きな値で線形化 | 小さな値（エラー）で線形化 |
| 数値安定性 | 中 | **高** |
| 計算効率 | 中 | **高**（エラー状態が小さい） |
| クォータニオン制約 | 毎回正規化必要 | **自然に保たれる** |
| 実装難易度 | 低 | 中（リセットが必要） |

## 10. 実装チェックリスト

- [ ] ノミナル状態の初期化
- [ ] エラー共分散の初期化
- [ ] ノミナル状態伝播関数
- [ ] システム行列 $F_c$ の計算
- [ ] エラー共分散伝播
- [ ] 各センサーの観測モデル
- [ ] 観測ヤコビアン $H$ の計算
- [ ] カルマンゲイン計算
- [ ] エラー状態更新
- [ ] **エラー状態のノミナル状態への注入**
- [ ] **エラー状態のリセット**
- [ ] クォータニオン正規化
- [ ] 外れ値検出機構

## 11. 参考文献

1. **Quaternion kinematics for the error-state Kalman filter** (Joan Solà, 2017)
   - ESKFの完全な数学的導出

2. **Indirect Kalman Filter for 3D Attitude Estimation** (Nikolas Trawny and Stergios I. Roumeliotis)
   - ESKFの理論的背景

---

**本資料は実装可能なレベルでESKFを完全に記述している。次のステップとして、C++/Pythonコードへの実装を推奨する。**
