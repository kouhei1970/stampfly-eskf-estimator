#pragma once

#include <cmath>
#include <iostream>

namespace stampfly {
namespace math {

/**
 * @brief 3次元ベクトルクラス
 *
 * このクラスは3次元空間のベクトルを表現します。
 * 教育目的のため、すべての演算を明示的に実装しています。
 *
 * 数学的表記: v = [x, y, z]^T
 *
 * 主な用途:
 * - 位置ベクトル (position)
 * - 速度ベクトル (velocity)
 * - 加速度ベクトル (acceleration)
 * - 角速度ベクトル (angular velocity)
 */
class Vector3 {
public:
    // ========== コンストラクタ ==========

    /**
     * @brief デフォルトコンストラクタ
     * すべての要素を0で初期化します
     */
    Vector3();

    /**
     * @brief 値を指定するコンストラクタ
     * @param x X成分
     * @param y Y成分
     * @param z Z成分
     */
    Vector3(double x, double y, double z);

    /**
     * @brief 配列から構築するコンストラクタ
     * @param data 3要素の配列
     */
    explicit Vector3(const double data[3]);

    // ========== アクセサ ==========

    /**
     * @brief 要素へのアクセス（読み書き可能）
     * @param index インデックス (0=x, 1=y, 2=z)
     * @return 要素への参照
     */
    double& operator[](int index);

    /**
     * @brief 要素へのアクセス（読み取り専用）
     * @param index インデックス (0=x, 1=y, 2=z)
     * @return 要素の値
     */
    double operator[](int index) const;

    /**
     * @brief X成分を取得
     */
    double x() const { return data_[0]; }

    /**
     * @brief Y成分を取得
     */
    double y() const { return data_[1]; }

    /**
     * @brief Z成分を取得
     */
    double z() const { return data_[2]; }

    /**
     * @brief X成分を設定
     */
    void setX(double x) { data_[0] = x; }

    /**
     * @brief Y成分を設定
     */
    void setY(double y) { data_[1] = y; }

    /**
     * @brief Z成分を設定
     */
    void setZ(double z) { data_[2] = z; }

    /**
     * @brief すべての成分を設定
     */
    void set(double x, double y, double z);

    // ========== ベクトル演算 ==========

    /**
     * @brief ベクトルの加算
     *
     * 数学的定義: c = a + b = [a.x + b.x, a.y + b.y, a.z + b.z]^T
     *
     * @param other 加算するベクトル
     * @return 加算結果の新しいベクトル
     */
    Vector3 operator+(const Vector3& other) const;

    /**
     * @brief ベクトルの減算
     *
     * 数学的定義: c = a - b = [a.x - b.x, a.y - b.y, a.z - b.z]^T
     *
     * @param other 減算するベクトル
     * @return 減算結果の新しいベクトル
     */
    Vector3 operator-(const Vector3& other) const;

    /**
     * @brief スカラー倍
     *
     * 数学的定義: c = k * a = [k*a.x, k*a.y, k*a.z]^T
     *
     * @param scalar スカラー値
     * @return スカラー倍された新しいベクトル
     */
    Vector3 operator*(double scalar) const;

    /**
     * @brief スカラー除算
     *
     * 数学的定義: c = a / k = [a.x/k, a.y/k, a.z/k]^T
     *
     * @param scalar スカラー値（0でないこと）
     * @return スカラー除算された新しいベクトル
     */
    Vector3 operator/(double scalar) const;

    /**
     * @brief 単項マイナス演算子
     *
     * 数学的定義: -a = [-a.x, -a.y, -a.z]^T
     *
     * @return 符号を反転した新しいベクトル
     */
    Vector3 operator-() const;

    /**
     * @brief 加算代入演算子
     * this = this + other
     */
    Vector3& operator+=(const Vector3& other);

    /**
     * @brief 減算代入演算子
     * this = this - other
     */
    Vector3& operator-=(const Vector3& other);

    /**
     * @brief スカラー倍代入演算子
     * this = this * scalar
     */
    Vector3& operator*=(double scalar);

    /**
     * @brief スカラー除算代入演算子
     * this = this / scalar
     */
    Vector3& operator/=(double scalar);

    // ========== 内積・外積 ==========

    /**
     * @brief 内積（ドット積）
     *
     * 数学的定義: a · b = a.x*b.x + a.y*b.y + a.z*b.z
     *
     * 幾何学的意味: a · b = |a| |b| cos(θ)
     * ここで θ は2つのベクトルのなす角
     *
     * 用途:
     * - ベクトルの直交性判定（a·b = 0 なら直交）
     * - ベクトルの射影計算
     * - なす角の計算
     *
     * @param other もう一方のベクトル
     * @return 内積の値
     */
    double dot(const Vector3& other) const;

    /**
     * @brief 外積（クロス積）
     *
     * 数学的定義:
     * a × b = [a.y*b.z - a.z*b.y,
     *          a.z*b.x - a.x*b.z,
     *          a.x*b.y - a.y*b.x]^T
     *
     * 幾何学的意味:
     * - 結果はa, bの両方に垂直なベクトル
     * - 大きさは |a × b| = |a| |b| sin(θ)
     * - 右手系に従う（aからbへ右ねじ方向）
     *
     * 用途:
     * - 回転軸の計算
     * - 面積の計算
     * - 座標系の構築
     *
     * @param other もう一方のベクトル
     * @return 外積の結果ベクトル
     */
    Vector3 cross(const Vector3& other) const;

    // ========== ノルム・正規化 ==========

    /**
     * @brief ベクトルの長さ（ノルム）を計算
     *
     * 数学的定義: |a| = √(a.x² + a.y² + a.z²)
     *
     * @return ベクトルの長さ
     */
    double norm() const;

    /**
     * @brief ベクトルの長さの2乗を計算
     *
     * 数学的定義: |a|² = a.x² + a.y² + a.z²
     *
     * 注意: sqrt()を呼ばないため、norm()より高速です
     * 長さの比較などに使用できます
     *
     * @return ベクトルの長さの2乗
     */
    double squaredNorm() const;

    /**
     * @brief 正規化されたベクトルを返す
     *
     * 数学的定義: â = a / |a|
     *
     * 正規化されたベクトル（単位ベクトル）は長さが1です。
     * 方向は元のベクトルと同じです。
     *
     * 用途:
     * - 方向のみを表現したい場合
     * - 回転軸として使用する場合
     *
     * @return 正規化された新しいベクトル（長さ1）
     */
    Vector3 normalized() const;

    /**
     * @brief このベクトルを正規化する（インプレース）
     *
     * 元のベクトルを直接変更します
     */
    void normalize();

    /**
     * @brief ゼロベクトルかどうか判定
     *
     * すべての成分がほぼ0の場合にtrueを返します
     *
     * @param epsilon 判定閾値（デフォルト: 1e-9）
     * @return ゼロベクトルならtrue
     */
    bool isZero(double epsilon = 1e-9) const;

    // ========== ユーティリティ ==========

    /**
     * @brief デバッグ用の文字列表現
     *
     * 例: "[1.000, 2.000, 3.000]"
     *
     * @return ベクトルの文字列表現
     */
    std::string toString() const;

    /**
     * @brief 標準出力への出力
     */
    friend std::ostream& operator<<(std::ostream& os, const Vector3& v);

    /**
     * @brief スカラー × ベクトル（順序逆）
     *
     * k * v と v * k の両方をサポート
     */
    friend Vector3 operator*(double scalar, const Vector3& v);

    // ========== 静的メソッド ==========

    /**
     * @brief ゼロベクトルを生成
     * @return [0, 0, 0]
     */
    static Vector3 zero();

    /**
     * @brief X軸単位ベクトルを生成
     * @return [1, 0, 0]
     */
    static Vector3 unitX();

    /**
     * @brief Y軸単位ベクトルを生成
     * @return [0, 1, 0]
     */
    static Vector3 unitY();

    /**
     * @brief Z軸単位ベクトルを生成
     * @return [0, 0, 1]
     */
    static Vector3 unitZ();

private:
    double data_[3];  ///< ベクトルの要素 [x, y, z]
};

} // namespace math
} // namespace stampfly
