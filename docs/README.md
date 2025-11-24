# StampFly ESKF Estimator - ドキュメント

このディレクトリには、ESKFベースの位置・姿勢推定システムに関する完全なドキュメントとリファレンス実装が含まれています。

## 📚 構成

```
docs/
├── theory/              # 理論・アルゴリズム解説
│   ├── EKF_Position_Estimation.md     # 標準EKF完全設計書
│   └── ESKF_Position_Estimation.md    # ESKF完全設計書（推奨）
└── reference_impl/      # リファレンス実装
    ├── python/          # Python実装（教育・プロトタイピング用）
    └── cpp/             # C++実装（教育・検証用）
```

## 📖 理論ドキュメント

### [EKF_Position_Estimation.md](theory/EKF_Position_Estimation.md)

**標準Extended Kalman Filter (EKF) の完全設計書**

- ✅ 状態ベクトル定義（15次元）
- ✅ 完全な数式導出（LaTeX形式）
- ✅ すべてのセンサーの観測モデル
- ✅ ヤコビ行列の明示的な導出
- ✅ パラメータチューニングガイド
- ✅ 実装ガイドライン

**対象読者:**
- カルマンフィルタの基礎を学びたい方
- シンプルな実装から始めたい方
- ESKFの前段階として理解したい方

### [ESKF_Position_Estimation.md](theory/ESKF_Position_Estimation.md)

**Error-State Kalman Filter (ESKF) の完全設計書**

- ✅ **独立完結型**: この文書だけで完全に理解可能
- ✅ ESKFの理論的背景と利点
- ✅ ノミナル状態とエラー状態の定義
- ✅ エラー状態の注入とリセット
- ✅ 数値的安定性の説明
- ✅ 完全な実装アルゴリズム

**対象読者:**
- **実機実装を行う方（推奨）**
- 高精度な推定が必要な方

## 💻 リファレンス実装

### Python実装 ([reference_impl/python/](reference_impl/python/))

**特徴:**
- NumPyベースのクリーンな実装
- プロトタイピングに最適
- 可読性重視のコード
- Jupyter Notebookでの検証に適している

**含まれるファイル:**
- `ekf_position_estimator.py` - 標準EKF実装
- （ESKFのPython実装は今後追加予定）

**使用方法:**
```python
import numpy as np
from ekf_position_estimator import EKFPositionEstimator, EKFParameters

params = EKFParameters()
ekf = EKFPositionEstimator(params)

# センサーデータ処理
ekf.predict(a_meas, w_meas, dt)
ekf.update_barometer(altitude)
```

### C++実装 ([reference_impl/cpp/](reference_impl/cpp/))

**特徴:**
- Eigen3ベースの効率的な実装
- 教育・検証目的
- 理論ドキュメントと1対1対応
- クリーンで読みやすいコード

**含まれるファイル:**
- `ekf_position_estimator.hpp/cpp` - 標準EKF実装
- `eskf_position_estimator.hpp/cpp` - ESKF実装
- `CMakeLists.txt` - ビルド設定

**使用方法:**
```cpp
#include "eskf_position_estimator.hpp"
using namespace quadcopter;

ESKFParameters params;
ESKFPositionEstimator eskf(params);

// センサーデータ処理
eskf.predict(a_meas, w_meas, dt);
eskf.updateBarometer(altitude);
```

## 🎓 学習パス

### 初学者向け

1. **基礎を学ぶ**: [EKF_Position_Estimation.md](theory/EKF_Position_Estimation.md) を読む
2. **Pythonで実験**: [reference_impl/python/](reference_impl/python/) で動作確認
3. **ESKFへ**: [ESKF_Position_Estimation.md](theory/ESKF_Position_Estimation.md) でESKFを理解
4. **C++で実装**: [reference_impl/cpp/](reference_impl/cpp/) を参照して実装

### 実装者向け（実機開発）

1. **ESKFを直接学ぶ**: [ESKF_Position_Estimation.md](theory/ESKF_Position_Estimation.md)（独立完結型）
2. **リファレンス実装を確認**: [reference_impl/cpp/](reference_impl/cpp/)
3. **本番API使用**: [`include/position_estimator/`](../include/position_estimator/) のヘッダー
4. **サンプル参照**: [`examples/`](../examples/) で使用方法を確認

## 📝 ドキュメントの読み方

### 数式表記

すべての数式は **LaTeX形式** で記述されています：

```markdown
状態ベクトル：
$
\mathbf{x} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \mathbf{q} \end{bmatrix}
$
```

数式環境内には `\text{}` によるコメントは含まれていません（フォント統一のため）。
コメントはすべて数式の外に記述されています。

### 座標系規約

すべてのドキュメントで **NED座標系（North-East-Down）** を使用：

- **ワールド座標**: x=北、y=東、z=下
- **ボディ座標**: x=前、y=右、z=下

### 記号の統一

| 記号 | 意味 |
|------|------|
| $\mathbf{p}$ | 位置（World frame） |
| $\mathbf{v}$ | 速度（World frame） |
| $\mathbf{q}$ | クォータニオン（姿勢） |
| $\mathbf{b}_g$ | ジャイロバイアス |
| $\mathbf{b}_a$ | 加速度バイアス |
| $\delta\mathbf{x}$ | エラー状態（ESKF） |
| $\mathbf{Q}$ | プロセスノイズ共分散 |
| $\mathbf{R}$ | 観測ノイズ共分散 |
| $\mathbf{P}$ | 状態共分散 |

## 🔍 リファレンス実装 vs 本番実装

### リファレンス実装（このディレクトリ）

**目的:**
- 教育・学習
- アルゴリズム検証
- プロトタイピング

**特徴:**
- 可読性最優先
- 理論ドキュメントと完全対応
- 最適化よりも理解しやすさ重視

### 本番実装（`../include/`, `../src/`）

**目的:**
- 実機システムでの使用
- 高パフォーマンス
- 保守性・拡張性

**特徴:**
- モジュラー設計
- センサー抽象化
- 効率的なメモリ管理
- 実用的なエラーハンドリング

## 🤝 貢献

ドキュメントの改善提案・誤りの報告は大歓迎です！

- 誤字・脱字の修正
- 説明の明確化
- サンプルコードの追加
- 新しい実装例

Pull Requestまたは Issueでお知らせください。

## 📚 参考文献

1. **Quaternion kinematics for the error-state Kalman filter**
   - Joan Solà, 2017
   - ESKFの包括的な数学的解説

2. **Indirect Kalman Filter for 3D Attitude Estimation**
   - Nikolas Trawny and Stergios I. Roumeliotis
   - 理論的基盤

## ⚠️ 注意事項

- リファレンス実装は教育目的です
- 実機での使用は本番実装（`../include/`）を推奨
- パラメータは実際のセンサーに合わせて調整が必要
- 安全のため、十分なテストを実施してください
