# リファレンス実装（C++ / Eigen3版）

⚠️ **注意**: このディレクトリのコードは **Eigen3ライブラリを使用した参考実装** です。

## 📌 位置づけ

このリファレンス実装は以下の目的で保持されています：

1. **理論ドキュメントとの対応**: `docs/theory/` の数学的定義を直接的に実装
2. **比較・検証用**: 独自実装版（`include/stampfly_math/`）の正しさを検証
3. **学習の足がかり**: 線形代数ライブラリ（Eigen3）を使った標準的な実装方法を理解

## 🆚 独自実装版との違い

| 項目 | このディレクトリ（参考） | 本体（`include/`, `src/`） |
|------|----------------------|--------------------------|
| **ライブラリ** | Eigen3 | stampfly_math（独自実装） |
| **目的** | 理論の直接実装・検証 | 教育・学習 |
| **可読性** | 高い（Eigen3の簡潔な記法） | 非常に高い（すべて明示的） |
| **学習効果** | 中（ライブラリの使い方） | 高（内部の仕組みまで理解） |
| **推奨用途** | 理論検証、プロトタイピング | 学習、教育、研究 |

## 📂 ファイル

| ファイル | 説明 |
|---------|------|
| `ekf_position_estimator.hpp/cpp` | 標準EKF実装（Eigen3使用） |
| `eskf_position_estimator.hpp/cpp` | ESKF実装（Eigen3使用） |
| `CMakeLists.txt` | ビルド設定（Eigen3必須） |

## 🎓 推奨される学習フロー

### 初学者向け

1. **まず理論を読む**: `docs/theory/ESKF_Position_Estimation.md`
2. **このEigen3版を見る**: ライブラリを使った標準的な実装を理解
3. **独自実装を読む**: `include/stampfly_math/` で内部の仕組みを学ぶ
4. **サンプルを実行**: `examples/` で実際の動作を確認

### 中級者・実装者向け

1. **独自実装を先に読む**: 内部の仕組みを完全に理解
2. **このEigen3版と比較**: ライブラリを使うと何が簡潔になるか理解
3. **本番システムへ**: 実機に合わせて `include/` のAPIを使用

## 🔧 ビルド方法

Eigen3が必要です：

```bash
# Ubuntu/Debian
sudo apt-get install libeigen3-dev

# macOS
brew install eigen
```

ビルド:

```bash
mkdir build && cd build
cmake .. -DBUILD_REFERENCE_IMPL=ON
make
```

## 💡 コードの特徴

### Eigen3版の例
```cpp
// ベクトルの内積（Eigen3）
double dot_product = v1.dot(v2);

// 行列の逆行列（Eigen3）
MatrixXd P_inv = P.inverse();

// 行列乗算（Eigen3）
MatrixXd result = H * P * H.transpose();
```

### 独自実装版の例
```cpp
// ベクトルの内積（stampfly_math）
double dot_product = v1.dot(v2);  // 同じAPI！

// 行列の逆行列（stampfly_math）
Matrix P_inv = P.inverse();  // 実装が明示的に見える

// 行列乗算（stampfly_math）
Matrix result = H * P * H.transpose();  // すべての演算が追跡可能
```

## ⚠️ 注意事項

- このディレクトリのコードは **教育・研究用** です
- 実機での使用は **独自実装版**（`include/`, `src/`）を推奨
- Eigen3がない環境でも、独自実装版は動作します
- パラメータ調整は実際のセンサーに合わせて行ってください

## 📚 関連ドキュメント

- [ESKF理論](../../theory/ESKF_Position_Estimation.md) - 数学的な定義
- [独自実装](../../../include/stampfly_math/) - 教育用実装（推奨）
- [サンプルコード](../../../examples/) - 実際の使用例

---

**このコードはEigen3を使用した参考実装です。学習には独自実装版（`stampfly_math`）をお勧めします。**
