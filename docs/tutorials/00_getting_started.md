# 入門チュートリアル - はじめての StampFly ESKF Estimator

このチュートリアルでは、プログラミング初心者でも StampFly ESKF Estimator を使い始められるよう、基礎から丁寧に説明します。

## 🎯 このチュートリアルで学ぶこと

- 開発環境の構築
- プロジェクトのビルド方法
- 最初のプログラムの実行
- Vector3（3次元ベクトル）の基本操作

## 📋 必要な前提知識

- ✅ 基本的なC++の知識（変数、関数、クラスの概念）
- ✅ ターミナル（コマンドライン）の基本操作
- ❌ 線形代数の深い知識は不要
- ❌ CMakeの詳しい知識は不要
- ❌ カルマンフィルタの知識は不要

## ステップ1: 開発環境の構築

### macOS の場合

```bash
# Xcodeコマンドラインツールのインストール
xcode-select --install

# Homebrewがない場合はインストール
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# CMakeのインストール
brew install cmake
```

### Ubuntu/Debian の場合

```bash
# 必要なツールをインストール
sudo apt-get update
sudo apt-get install build-essential cmake git
```

### Windows の場合

1. Visual Studio 2019以降をインストール（Community版で可）
2. CMakeをインストール: https://cmake.org/download/
3. Git Bashをインストール: https://gitforwindows.org/

## ステップ2: プロジェクトの取得

```bash
# リポジトリをクローン
git clone https://github.com/kouhei1970/stampfly-eskf-estimator.git

# ディレクトリに移動
cd stampfly-eskf-estimator

# ファイル構成を確認
ls
```

以下のようなディレクトリ構造が見えるはずです：
```
docs/       # ドキュメント
examples/   # サンプルコード
include/    # ヘッダーファイル
src/        # 実装
config/     # 設定ファイル
CMakeLists.txt
README.md
```

## ステップ3: プロジェクトのビルド

```bash
# ビルド用ディレクトリを作成
mkdir build
cd build

# CMakeで設定（リファレンス実装はスキップ）
cmake .. -DBUILD_REFERENCE_IMPL=OFF

# ビルド実行
make

# または並列ビルド（速い）
make -j4
```

**成功すると以下のようなメッセージが表示されます：**
```
[100%] Built target 01_vector_operations
[100%] Built target 02_quaternion_rotation
```

## ステップ4: 最初のプログラムを実行

```bash
# Vector3の基本操作を実行
./examples/01_vector_operations
```

**出力例：**
```
========================================
  Vector3 クラス - 基本操作
========================================

【1】ベクトルの作成
--------------------
v1 = [1.000, 2.000, 3.000]
v2 = [4.000, 5.000, 6.000]

【2】基本演算
--------------------
v1 + v2 = [5.000, 7.000, 9.000]
...
```

おめでとうございます！最初のプログラムが動きました！🎉

## ステップ5: コードを理解する

サンプルコードを見てみましょう：

```cpp
#include "stampfly_math/vector3.hpp"
#include <iostream>

using namespace stampfly::math;

int main() {
    // ベクトルの作成
    Vector3 v1(1.0, 2.0, 3.0);  // x=1, y=2, z=3
    Vector3 v2(4.0, 5.0, 6.0);

    // 加算
    Vector3 sum = v1 + v2;
    std::cout << "v1 + v2 = " << sum << std::endl;

    return 0;
}
```

**解説：**
1. `#include "stampfly_math/vector3.hpp"` - Vector3クラスを使用
2. `Vector3 v1(1.0, 2.0, 3.0)` - 3次元ベクトルを作成
3. `v1 + v2` - 演算子オーバーロードで直感的に加算
4. `std::cout << sum` - 出力演算子も実装済み

## ステップ6: 自分でコードを書いてみる

`examples/` ディレクトリに新しいファイル `my_first_program.cpp` を作成：

```cpp
#include "stampfly_math/vector3.hpp"
#include <iostream>

using namespace stampfly::math;

int main() {
    // あなたのコードをここに！

    // 例：2つのベクトルの内積を計算
    Vector3 a(1.0, 0.0, 0.0);
    Vector3 b(0.0, 1.0, 0.0);

    double dot_product = a.dot(b);
    std::cout << "内積: " << dot_product << std::endl;

    // 外積も試してみよう
    Vector3 cross_product = a.cross(b);
    std::cout << "外積: " << cross_product << std::endl;

    return 0;
}
```

**ビルド方法：**
```bash
# buildディレクトリで
g++ -std=c++14 -I../include ../examples/my_first_program.cpp ../src/stampfly_math/*.cpp -o my_program

# 実行
./my_program
```

## 📚 次のステップ

✅ できたこと：
- 開発環境の構築
- プロジェクトのビルド
- サンプルプログラムの実行
- Vector3の基本操作

🎯 次に学ぶこと：
1. [02_quaternion_rotation.cpp](../../examples/02_quaternion_rotation.cpp) - 回転の表現
2. [ESKF理論](../theory/ESKF_Position_Estimation.md) - カルマンフィルタの数学
3. 独自の推定器を実装

## ❓ トラブルシューティング

### CMakeが見つからない
```bash
# CMakeのインストールを確認
cmake --version

# ない場合は再インストール
```

### コンパイルエラーが出る
- C++14以降に対応したコンパイラか確認
- `g++ --version` または `clang --version`

### Eigen3が必要と言われる
- リファレンス実装をスキップ: `cmake .. -DBUILD_REFERENCE_IMPL=OFF`

## 💬 質問・フィードバック

- GitHubでIssueを作成: https://github.com/kouhei1970/stampfly-eskf-estimator/issues
- ドキュメントの改善提案も歓迎！

---

**次のチュートリアル:** [01_linear_algebra_basics.md](01_linear_algebra_basics.md)（今後追加予定）
