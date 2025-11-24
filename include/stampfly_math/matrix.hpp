#pragma once

#include "vector3.hpp"
#include <vector>
#include <string>
#include <iostream>

namespace stampfly {
namespace math {

/**
 * @brief 汎用行列クラス（動的サイズ）
 *
 * このクラスは任意サイズの行列を表現します。
 * 教育目的のため、すべての演算を明示的に実装しています。
 *
 * 数学的表記: A ∈ ℝ^(m×n) (m行n列の実数行列)
 *
 * 主な用途:
 * - カルマンフィルタの共分散行列 P
 * - ヤコビ行列 H, F
 * - ゲイン行列 K
 * - プロセスノイズ Q, 観測ノイズ R
 *
 * メモリレイアウト: 行優先（row-major）
 * A[i][j] = data_[i * cols + j]
 */
class Matrix {
public:
    // ========== コンストラクタ ==========

    /**
     * @brief デフォルトコンストラクタ
     * 空の行列（0×0）を作成
     */
    Matrix();

    /**
     * @brief サイズ指定コンストラクタ
     * 指定サイズの行列を作成し、0で初期化
     * @param rows 行数
     * @param cols 列数
     */
    Matrix(int rows, int cols);

    /**
     * @brief サイズと初期値指定コンストラクタ
     * @param rows 行数
     * @param cols 列数
     * @param value すべての要素の初期値
     */
    Matrix(int rows, int cols, double value);

    /**
     * @brief データから構築
     * @param rows 行数
     * @param cols 列数
     * @param data 行優先形式のデータ（rows × cols 個）
     */
    Matrix(int rows, int cols, const std::vector<double>& data);

    // ========== サイズ・形状 ==========

    /**
     * @brief 行数を取得
     */
    int rows() const { return rows_; }

    /**
     * @brief 列数を取得
     */
    int cols() const { return cols_; }

    /**
     * @brief 行列のサイズ（要素数）
     */
    int size() const { return rows_ * cols_; }

    /**
     * @brief 正方行列かどうか判定
     */
    bool isSquare() const { return rows_ == cols_; }

    /**
     * @brief 行列のリサイズ
     * @param rows 新しい行数
     * @param cols 新しい列数
     * 注意: 既存データは破棄され、0で初期化されます
     */
    void resize(int rows, int cols);

    // ========== 要素アクセス ==========

    /**
     * @brief (i,j)要素へのアクセス（読み書き可能）
     * @param i 行インデックス（0-based）
     * @param j 列インデックス（0-based）
     */
    double& operator()(int i, int j);

    /**
     * @brief (i,j)要素へのアクセス（読み取り専用）
     */
    double operator()(int i, int j) const;

    /**
     * @brief 要素を設定
     */
    void set(int i, int j, double value);

    /**
     * @brief 要素を取得
     */
    double get(int i, int j) const;

    // ========== 行列演算 ==========

    /**
     * @brief 行列の加算
     *
     * 数学的定義: C = A + B
     * C[i][j] = A[i][j] + B[i][j]
     *
     * @param other 加算する行列（同じサイズであること）
     * @return 加算結果の新しい行列
     */
    Matrix operator+(const Matrix& other) const;

    /**
     * @brief 行列の減算
     *
     * 数学的定義: C = A - B
     * C[i][j] = A[i][j] - B[i][j]
     */
    Matrix operator-(const Matrix& other) const;

    /**
     * @brief 行列の乗算
     *
     * 数学的定義: C = A * B
     * C[i][j] = Σ_k A[i][k] * B[k][j]
     *
     * 注意: A が m×n, B が n×p のとき、C は m×p
     *
     * @param other 乗算する行列（this->cols == other.rows であること）
     * @return 乗算結果の新しい行列
     */
    Matrix operator*(const Matrix& other) const;

    /**
     * @brief スカラー倍
     *
     * 数学的定義: C = k * A
     * C[i][j] = k * A[i][j]
     */
    Matrix operator*(double scalar) const;

    /**
     * @brief スカラー除算
     */
    Matrix operator/(double scalar) const;

    /**
     * @brief 単項マイナス
     */
    Matrix operator-() const;

    /**
     * @brief 加算代入
     */
    Matrix& operator+=(const Matrix& other);

    /**
     * @brief 減算代入
     */
    Matrix& operator-=(const Matrix& other);

    /**
     * @brief スカラー倍代入
     */
    Matrix& operator*=(double scalar);

    // ========== 転置・逆行列 ==========

    /**
     * @brief 転置行列
     *
     * 数学的定義: B = A^T
     * B[i][j] = A[j][i]
     *
     * m×n 行列の転置は n×m 行列
     *
     * @return 転置行列
     */
    Matrix transpose() const;

    /**
     * @brief 逆行列
     *
     * 数学的定義: A^(-1) は A * A^(-1) = I を満たす行列
     *
     * 注意:
     * - 正方行列のみ
     * - 特異行列（行列式=0）は逆行列を持たない
     *
     * アルゴリズム: Gauss-Jordan 消去法
     *
     * @return 逆行列
     */
    Matrix inverse() const;

    /**
     * @brief 行列式
     *
     * 正方行列のみ計算可能
     * 行列が可逆かどうかの判定に使用（det(A) ≠ 0 なら可逆）
     *
     * @return 行列式の値
     */
    double determinant() const;

    // ========== 特殊な行列 ==========

    /**
     * @brief 単位行列を生成
     *
     * 数学的定義: I[i][j] = δ_ij (Kronecker delta)
     * 対角要素が1、それ以外が0
     *
     * @param n サイズ（n×n）
     * @return n×n 単位行列
     */
    static Matrix identity(int n);

    /**
     * @brief ゼロ行列を生成
     *
     * すべての要素が0の行列
     *
     * @param rows 行数
     * @param cols 列数
     * @return rows×cols ゼロ行列
     */
    static Matrix zeros(int rows, int cols);

    /**
     * @brief すべての要素が1の行列
     */
    static Matrix ones(int rows, int cols);

    /**
     * @brief 対角行列を生成
     *
     * 対角要素に指定された値を持つ行列
     *
     * @param diag 対角要素のベクトル
     * @return n×n 対角行列
     */
    static Matrix diagonal(const std::vector<double>& diag);

    // ========== ブロック操作 ==========

    /**
     * @brief 部分行列を抽出
     *
     * @param row_start 開始行
     * @param row_count 行数
     * @param col_start 開始列
     * @param col_count 列数
     * @return 抽出された部分行列
     */
    Matrix block(int row_start, int row_count, int col_start, int col_count) const;

    /**
     * @brief 部分行列を設定
     *
     * @param row_start 開始行
     * @param col_start 開始列
     * @param block 設定する部分行列
     */
    void setBlock(int row_start, int col_start, const Matrix& block);

    /**
     * @brief 特定の行を抽出
     */
    Matrix row(int i) const;

    /**
     * @brief 特定の列を抽出
     */
    Matrix col(int j) const;

    /**
     * @brief 行を設定
     */
    void setRow(int i, const Matrix& row_vector);

    /**
     * @brief 列を設定
     */
    void setCol(int j, const Matrix& col_vector);

    // ========== ユーティリティ ==========

    /**
     * @brief すべての要素を指定値で埋める
     */
    void fill(double value);

    /**
     * @brief すべての要素をゼロに設定
     */
    void setZero();

    /**
     * @brief 単位行列に設定（正方行列のみ）
     */
    void setIdentity();

    /**
     * @brief トレース（対角要素の和）
     *
     * 数学的定義: tr(A) = Σ_i A[i][i]
     *
     * @return トレース
     */
    double trace() const;

    /**
     * @brief フロベニウスノルム（行列の「大きさ」）
     *
     * 数学的定義: ||A||_F = √(Σ_i Σ_j A[i][j]²)
     *
     * @return フロベニウスノルム
     */
    double norm() const;

    /**
     * @brief デバッグ用文字列表現
     */
    std::string toString() const;

    /**
     * @brief 標準出力への出力
     */
    friend std::ostream& operator<<(std::ostream& os, const Matrix& m);

    /**
     * @brief スカラー × 行列
     */
    friend Matrix operator*(double scalar, const Matrix& m);

    // ========== Vector3 との相互運用 ==========

    /**
     * @brief Vector3 を 3×1 行列に変換
     */
    static Matrix fromVector3(const Vector3& v);

    /**
     * @brief 3×1 行列を Vector3 に変換
     */
    Vector3 toVector3() const;

private:
    int rows_;                    ///< 行数
    int cols_;                    ///< 列数
    std::vector<double> data_;    ///< データ（行優先）

    /**
     * @brief インデックスのバリデーション
     */
    void checkBounds(int i, int j) const;

    /**
     * @brief サイズの互換性チェック
     */
    void checkSizeMatch(const Matrix& other) const;
};

} // namespace math
} // namespace stampfly
