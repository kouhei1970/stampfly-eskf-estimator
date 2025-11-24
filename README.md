# StampFly ESKF Estimator

**教育・研究目的のオープンソースプロジェクト**

クアッドコプター「StampFly」向けの3次元位置・姿勢推定システム。Error-State Kalman Filter (ESKF) を **独自実装の数学ライブラリ** で実現し、内部の仕組みまで完全に理解できるように設計されています。

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 🎓 このプロジェクトで学べること

- ✅ **カルマンフィルタの仕組み**: 理論から実装まで完全に追跡可能
- ✅ **3次元回転の数学**: クォータニオン、回転行列、オイラー角
- ✅ **センサーフュージョン**: 複数のセンサーを統合する方法
- ✅ **C++プログラミング**: オブジェクト指向設計とベストプラクティス
- ✅ **数値計算**: 行列演算、逆行列、数値安定性

## 💡 なぜこのプロジェクト？

多くの実装は外部ライブラリ（Eigen、NumPyなど）に依存しており、**内部で何が起きているか見えません**。

このプロジェクトは：
- 📖 **すべて自前で実装**: Vector3、Quaternion、Matrix クラスを独自実装
- 💬 **詳細なコメント**: すべての演算に数式と説明付き
- 🎯 **教育重視**: 理解しやすさを最優先
- 🔬 **比較可能**: Eigen3版も併せて提供（違いが分かる）

## ✨ 特徴

- 🎯 **Error-State Kalman Filter (ESKF)**: 数値的に安定した高精度推定
- 📡 **マルチセンサーフュージョン**: IMU、地磁気、気圧、ToF、オプティカルフロー
- 🏗️ **独自数学ライブラリ**: 外部依存なし（Eigen不要）
- 📚 **完全なドキュメント**: 理論から実装まで詳細に解説
- 🔬 **段階的サンプル**: ベクトル演算から完全なESKFまで
- 🆚 **比較用リファレンス**: Eigen3版も提供（学習目的）

## 👥 対象者

### 🌱 プログラミング初心者
- 環境構築から丁寧に説明
- 基本的なC++の知識があれば OK
- CMakeの使い方も学べます

### 🎓 学生・研究者
- カルマンフィルタの内部を完全理解
- 論文実装の参考に
- 独自の拡張も容易

### 🚁 ドローン開発者
- 実機に使える設計
- センサーフュージョンの実装例
- パラメータチューニングガイド付き

## 📁 プロジェクト構成

```
stampfly-eskf-estimator/
├── include/stampfly_math/     # 🌟 独自実装数学ライブラリ（Eigen不要）
│   ├── vector3.hpp           #    3次元ベクトル
│   ├── quaternion.hpp        #    クォータニオン（回転）
│   ├── matrix.hpp            #    汎用行列クラス
│   └── math_utils.hpp        #    ユーティリティ関数
├── src/stampfly_math/         # 数学ライブラリの実装
├── docs/                      # ドキュメント
│   ├── theory/               # 理論・アルゴリズム解説
│   │   ├── EKF_Position_Estimation.md    # 標準EKF完全設計書
│   │   └── ESKF_Position_Estimation.md   # ESKF完全設計書
│   └── reference_impl/       # リファレンス実装（比較用）
│       ├── python/           # Python実装（NumPy使用）
│       └── cpp/              # C++実装（Eigen3使用）
├── examples/                  # 段階的なサンプルコード
│   ├── 01_vector_operations.cpp       # Vector3の基本
│   ├── 02_quaternion_rotation.cpp     # Quaternionの使い方
│   └── ... (今後追加)
├── config/                    # 設定ファイル
│   ├── default_params.yaml   # デフォルトパラメータ
│   ├── indoor_params.yaml    # 室内飛行用
│   └── outdoor_params.yaml   # 屋外飛行用
└── tests/                     # テストコード（今後追加）
```

### 🌟 独自実装数学ライブラリ（stampfly_math）

**外部依存ゼロ！すべて自前実装！**

| クラス | ファイル | 説明 |
|-------|---------|------|
| Vector3 | [include/stampfly_math/vector3.hpp](include/stampfly_math/vector3.hpp) | 3次元ベクトル演算（内積、外積、正規化など） |
| Quaternion | [include/stampfly_math/quaternion.hpp](include/stampfly_math/quaternion.hpp) | 回転表現（オイラー角変換、SLERP補間など） |
| Matrix | [include/stampfly_math/matrix.hpp](include/stampfly_math/matrix.hpp) | 汎用行列（逆行列、転置、乗算など） |
| ユーティリティ | [include/stampfly_math/math_utils.hpp](include/stampfly_math/math_utils.hpp) | スキュー対称行列、マハラノビス距離など |

**特徴:**
- ✅ すべての演算が明示的に実装されている
- ✅ 詳細なコメントと数式の説明付き
- ✅ デバッグ出力機能
- ✅ 教育目的に最適化

### 📖 理論ドキュメント

| ドキュメント | 説明 |
|------------|------|
| [docs/theory/EKF_Position_Estimation.md](docs/theory/EKF_Position_Estimation.md) | 標準EKFの数学的定義 |
| [docs/theory/ESKF_Position_Estimation.md](docs/theory/ESKF_Position_Estimation.md) | ESKF完全設計書（推奨） |

### 🆚 リファレンス実装（比較・検証用）

| 実装 | ライブラリ | 用途 |
|------|-----------|------|
| [docs/reference_impl/python/](docs/reference_impl/python/) | NumPy | プロトタイピング、理論検証 |
| [docs/reference_impl/cpp/](docs/reference_impl/cpp/) | Eigen3 | 標準的な実装方法の参照 |

**注意:** これらはライブラリの使い方を学ぶための参考です。学習には独自実装（stampfly_math）を推奨します。

## 🎯 対応センサー

すべての実装で以下のセンサーをサポート：

- ✅ **IMU（慣性計測装置）**
  - 3軸加速度センサ
  - 3軸角速度センサ（ジャイロ）
- ✅ **地磁気センサ**（3軸磁気センサ）
- ✅ **オプティカルフローセンサ**
- ✅ **ToF（Time-of-Flight）高度センサ**
- ✅ **気圧高度センサ**

## 🔬 EKF vs ESKF

### 標準EKF（Extended Kalman Filter）

**特徴：**
- 状態そのものを推定
- 実装が比較的シンプル
- 教育・プロトタイピングに最適

**推奨用途：**
- 学習目的
- 小規模プロジェクト
- シミュレーション環境

### ESKF（Error-State Kalman Filter）

**特徴：**
- エラー状態（小さな誤差）を推定
- **数値安定性が高い**
- **姿勢推定の精度が高い**

**推奨用途：**
- **実機実装（推奨）**
- 高精度が必要な場合
- 長時間運用

### 比較表

| 項目 | 標準EKF | ESKF |
|------|---------|------|
| 数値安定性 | 中 | ⭐⭐⭐ 高 |
| 計算効率 | 中 | ⭐⭐ 高 |
| 実装難易度 | ⭐ 低 | 中 |
| 実機適性 | 中 | ⭐⭐⭐ 高 |
| 姿勢精度 | 中 | ⭐⭐⭐ 高 |

## 🚀 クイックスタート

### 🌱 初心者向け：まずはVector3から

**外部ライブラリ不要！すぐに試せます！**

```cpp
#include "stampfly_math/vector3.hpp"
#include <iostream>

using namespace stampfly::math;

int main() {
    // ベクトルの作成
    Vector3 v1(1.0, 2.0, 3.0);
    Vector3 v2(4.0, 5.0, 6.0);

    // 加算
    Vector3 sum = v1 + v2;
    std::cout << "v1 + v2 = " << sum << std::endl;

    // 内積
    double dot = v1.dot(v2);
    std::cout << "v1 · v2 = " << dot << std::endl;

    // 外積
    Vector3 cross = v1.cross(v2);
    std::cout << "v1 × v2 = " << cross << std::endl;

    return 0;
}
```

詳しくは [examples/01_vector_operations.cpp](examples/01_vector_operations.cpp) を参照！

### 🔄 回転を学ぶ：Quaternion

```cpp
#include "stampfly_math/quaternion.hpp"
#include "stampfly_math/vector3.hpp"

using namespace stampfly::math;

int main() {
    // Z軸周りに90度回転
    Vector3 axis = Vector3::unitZ();
    double angle = M_PI / 2.0;  // 90度
    Quaternion q = Quaternion::fromAxisAngle(axis, angle);

    // ベクトルを回転
    Vector3 v = Vector3::unitX();  // X軸方向 [1,0,0]
    Vector3 rotated = q.rotate(v);  // Y軸方向 [0,1,0] になる

    std::cout << "回転前: " << v << std::endl;
    std::cout << "回転後: " << rotated << std::endl;

    return 0;
}
```

詳しくは [examples/02_quaternion_rotation.cpp](examples/02_quaternion_rotation.cpp) を参照！

## 🔧 ビルド方法

### 📦 必要なもの

**外部ライブラリ不要！CMake とコンパイラだけ！**

- **CMake** 3.10 以降
- **C++14** 対応コンパイラ
  - Linux: GCC 5+ または Clang 3.4+
  - macOS: Xcode Command Line Tools
  - Windows: Visual Studio 2015+ または MinGW

```bash
# Ubuntu/Debian
sudo apt-get install cmake g++

# macOS
xcode-select --install
brew install cmake

# Arch Linux
sudo pacman -S cmake gcc
```

**注意:** Eigen3 は **不要** です！独自実装を使用します。
（リファレンス実装の比較用にビルドする場合のみ Eigen3 が必要）

### 🚀 ビルド手順

```bash
# 1. リポジトリのクローン
git clone https://github.com/kouhei1970/stampfly-eskf-estimator.git
cd stampfly-eskf-estimator

# 2. ビルドディレクトリ作成
mkdir build && cd build

# 3. CMake 設定
cmake ..

# 4. ビルド
make -j$(nproc)

# 5. サンプル実行
./examples/01_vector_operations
./examples/02_quaternion_rotation
```

### ⚙️ ビルドオプション

| オプション | デフォルト | 説明 |
|-----------|----------|------|
| `BUILD_REFERENCE_IMPL` | ON | リファレンス実装（Eigen3必要） |
| `BUILD_EXAMPLES` | ON | サンプルコード（Eigen不要） |
| `BUILD_TESTS` | OFF | テスト（今後追加） |

リファレンス実装をスキップ（Eigen3なしでビルド）:
```bash
cmake .. -DBUILD_REFERENCE_IMPL=OFF
make
```

### 🔌 プロジェクトへの統合

#### CMakeプロジェクトに組み込む

```cmake
# あなたのCMakeLists.txt
add_subdirectory(stampfly-eskf-estimator)

add_executable(my_drone main.cpp)
target_link_libraries(my_drone
    stampfly_math  # 独自数学ライブラリ
)
```

#### ヘッダーのインクルード

```cpp
#include "stampfly_math/vector3.hpp"
#include "stampfly_math/quaternion.hpp"
#include "stampfly_math/matrix.hpp"

// すぐに使えます！
stampfly::math::Vector3 v(1, 2, 3);
```

## 📊 パラメータ設定

パラメータは YAML 設定ファイルで管理します。使用環境に応じて適切な設定を選択してください。

### 設定ファイル

| ファイル | 用途 | 特徴 |
|---------|------|------|
| [config/default_params.yaml](config/default_params.yaml) | 汎用設定 | バランス型の標準パラメータ |
| [config/indoor_params.yaml](config/indoor_params.yaml) | 室内飛行 | オプティカルフロー重視、地磁気無効 |
| [config/outdoor_params.yaml](config/outdoor_params.yaml) | 屋外飛行 | 地磁気・気圧重視、GPS準備 |

### パラメータ読み込み

```cpp
// YAMLファイルから読み込み（将来実装）
ESKFParameters params;
params.loadFromYAML("config/indoor_params.yaml");

// または、プリセット使用
ESKFParameters params = ESKFParameters::createIndoorDefault();
```

### 主要パラメータ

#### プロセスノイズ（Q行列）

システムモデルの不確かさ。大きいほどセンサー測定を信頼。

```yaml
process_noise:
  sigma_a: 0.1          # 加速度ノイズ [m/s²]
  sigma_omega: 0.001    # 角速度ノイズ [rad/s]
  sigma_bg: 0.00005     # ジャイロバイアスドリフト [rad/s/√s]
  sigma_ba: 0.001       # 加速度バイアスドリフト [m/s²/√s]
```

#### センサーノイズ（R行列）

センサー測定の不確かさ。データシートを参考に設定。

```yaml
sensors:
  magnetometer:
    noise_std: 0.3      # 地磁気ノイズ
  barometer:
    noise_std: 1.0      # 気圧高度ノイズ [m]
  tof:
    noise_std: 0.05     # ToF距離ノイズ [m]
  optical_flow:
    noise_std: 1.0      # フローノイズ [rad/s]
```

### チューニング手順

1. **静止状態テスト**: 位置・姿勢の安定性確認
2. **高度テスト**: 気圧・ToF融合を確認
3. **水平移動テスト**: オプティカルフロー統合を確認
4. **回転テスト**: 姿勢推定精度を確認
5. **動的飛行テスト**: 全センサー統合を確認

詳細は [docs/theory/ESKF_Position_Estimation.md](docs/theory/ESKF_Position_Estimation.md) の「パラメータチューニング」セクションを参照してください。

## 📚 理論的背景

### 座標系

**NED座標系（North-East-Down）を使用:**

- **ワールド座標系**: 地面固定
  - x軸: 北方向
  - y軸: 東方向
  - z軸: 下方向（重力方向）

- **ボディ座標系**: 機体固定
  - x軸: 機体前方
  - y軸: 機体右方向
  - z軸: 機体下方向

### 状態ベクトル

両方のフィルタで同じ15次元の状態を推定：

$$
\mathbf{x} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \mathbf{q} \\ \mathbf{b}_g \\ \mathbf{b}_a \end{bmatrix} = \begin{bmatrix} \text{位置 (3)} \\ \text{速度 (3)} \\ \text{クォータニオン (4)} \\ \text{ジャイロバイアス (3)} \\ \text{加速度バイアス (2)} \end{bmatrix}
$$

**ESKFの場合:**
- ノミナル状態: 上記の15次元
- エラー状態: 15次元（姿勢誤差は3次元の回転ベクトル）

## 🔍 実装の特徴

### 数値安定性

- ✅ Joseph形式の共分散更新
- ✅ 対称性の強制
- ✅ クォータニオンの正規化
- ✅ 外れ値検出（マハラノビス距離）

### センサーフュージョン

- ✅ **密結合（Tightly-coupled）**: 生センサーデータを直接統合
- ✅ 非同期センサー更新対応
- ✅ センサー更新頻度の個別管理

### 実用的な機能

- ✅ 磁気擾乱対策
- ✅ オプティカルフロー高度依存性処理
- ✅ ToF傾斜補正
- ✅ 外れ値自動検出

## 📖 参考文献

1. **Quaternion kinematics for the error-state Kalman filter** (Joan Solà, 2017)
   - ESKFの完全な数学的導出

2. **Indirect Kalman Filter for 3D Attitude Estimation** (Nikolas Trawny and Stergios I. Roumeliotis)
   - ESKFの理論的背景

## 📝 ライセンス

MIT License

## 🤝 コントリビューション

改善提案・バグ報告は Issue または Pull Request でお願いします。

## ⚠️ 注意事項

- 実機での使用前に、シミュレーション環境で十分にテストしてください
- パラメータは使用するセンサーに応じて調整が必要です
- 安全のため、フェイルセーフ機能を必ず実装してください

---

**本実装は教育・研究目的で作成されました。実機での使用は自己責任でお願いします。**
