#pragma once

#include "vector3.hpp"
#include <string>

namespace stampfly {
namespace math {

/**
 * @brief クォータニオンクラス（四元数）
 *
 * クォータニオンは3次元回転を表現する数学的構造です。
 * 教育目的のため、すべての演算を明示的に実装しています。
 *
 * 数学的表記: q = w + xi + yj + zk = [w, x, y, z]
 * ここで i² = j² = k² = ijk = -1
 *
 * または: q = [w, v] where v = [x, y, z] は虚部ベクトル
 *
 * 正規化クォータニオン（|q| = 1）は回転を表します:
 * - w: cos(θ/2) (スカラー部、実部)
 * - [x,y,z]: sin(θ/2) * axis (ベクトル部、虚部)
 * ここで θ は回転角、axis は回転軸の単位ベクトル
 *
 * 主な利点:
 * - ジンバルロックがない
 * - 滑らかな補間が可能（SLERP）
 * - 数値的に安定
 * - 計算が効率的
 *
 * オイラー角との比較:
 * - オイラー角: 直感的だがジンバルロックあり
 * - クォータニオン: 直感的でないが数学的に優れる
 */
class Quaternion {
public:
    // ========== コンストラクタ ==========

    /**
     * @brief デフォルトコンストラクタ
     * 単位クォータニオン [1, 0, 0, 0] で初期化（回転なし）
     */
    Quaternion();

    /**
     * @brief 値を指定するコンストラクタ
     * @param w スカラー部（実部）
     * @param x ベクトル部 x成分（虚部）
     * @param y ベクトル部 y成分（虚部）
     * @param z ベクトル部 z成分（虚部）
     */
    Quaternion(double w, double x, double y, double z);

    /**
     * @brief スカラー部とベクトル部を指定するコンストラクタ
     * @param w スカラー部（実部）
     * @param v ベクトル部（虚部）
     */
    Quaternion(double w, const Vector3& v);

    /**
     * @brief 配列から構築
     * @param data 4要素の配列 [w, x, y, z]
     */
    explicit Quaternion(const double data[4]);

    // ========== アクセサ ==========

    /**
     * @brief 要素へのアクセス（読み書き可能）
     * @param index インデックス (0=w, 1=x, 2=y, 3=z)
     */
    double& operator[](int index);

    /**
     * @brief 要素へのアクセス（読み取り専用）
     */
    double operator[](int index) const;

    /**
     * @brief スカラー部（実部）を取得
     */
    double w() const { return data_[0]; }

    /**
     * @brief ベクトル部 x成分を取得
     */
    double x() const { return data_[1]; }

    /**
     * @brief ベクトル部 y成分を取得
     */
    double y() const { return data_[2]; }

    /**
     * @brief ベクトル部 z成分を取得
     */
    double z() const { return data_[3]; }

    /**
     * @brief ベクトル部を Vector3 として取得
     */
    Vector3 vec() const { return Vector3(data_[1], data_[2], data_[3]); }

    /**
     * @brief すべての成分を設定
     */
    void set(double w, double x, double y, double z);

    // ========== クォータニオン演算 ==========

    /**
     * @brief クォータニオンの乗算
     *
     * 数学的定義:
     * q1 * q2 = [w1*w2 - v1·v2, w1*v2 + w2*v1 + v1×v2]
     *
     * ここで q = [w, v] の形式
     *
     * 幾何学的意味:
     * - 回転の合成
     * - q1 * q2 は「まずq2で回転、次にq1で回転」
     * - 注意: 非可換（q1*q2 ≠ q2*q1）
     *
     * @param other もう一方のクォータニオン
     * @return 乗算結果の新しいクォータニオン
     */
    Quaternion operator*(const Quaternion& other) const;

    /**
     * @brief クォータニオンでベクトルを回転
     *
     * 数学的定義:
     * v' = q * v * q^(-1)
     *
     * ただし v を純虚数クォータニオン [0, v] として扱う
     *
     * 幾何学的意味:
     * - ベクトルをクォータニオンが表す回転で回転
     * - 結果のベクトルは同じ長さ
     *
     * @param v 回転させるベクトル
     * @return 回転後のベクトル
     */
    Vector3 rotate(const Vector3& v) const;

    /**
     * @brief スカラー倍
     */
    Quaternion operator*(double scalar) const;

    /**
     * @brief スカラー除算
     */
    Quaternion operator/(double scalar) const;

    /**
     * @brief クォータニオンの加算
     * 注意: 回転の合成ではない（数値計算で使用）
     */
    Quaternion operator+(const Quaternion& other) const;

    /**
     * @brief クォータニオンの減算
     * 注意: 回転の差ではない（数値計算で使用）
     */
    Quaternion operator-(const Quaternion& other) const;

    // ========== 共役・逆元 ==========

    /**
     * @brief クォータニオンの共役
     *
     * 数学的定義: q* = [w, -v]
     *
     * 幾何学的意味:
     * - 逆回転を表す（正規化クォータニオンの場合）
     * - q * q* = |q|²（スカラー）
     *
     * @return 共役クォータニオン
     */
    Quaternion conjugate() const;

    /**
     * @brief クォータニオンの逆元
     *
     * 数学的定義: q^(-1) = q* / |q|²
     *
     * 幾何学的意味:
     * - 逆回転
     * - q * q^(-1) = 単位クォータニオン
     *
     * 注意: 正規化クォータニオンの場合、inverse() = conjugate()
     *
     * @return 逆クォータニオン
     */
    Quaternion inverse() const;

    // ========== ノルム・正規化 ==========

    /**
     * @brief クォータニオンのノルム（長さ）
     *
     * 数学的定義: |q| = √(w² + x² + y² + z²)
     *
     * 回転を表すクォータニオンは正規化されている（|q| = 1）
     */
    double norm() const;

    /**
     * @brief ノルムの2乗
     */
    double squaredNorm() const;

    /**
     * @brief 正規化されたクォータニオンを返す
     *
     * 回転を表すクォータニオンは必ず正規化する必要があります
     */
    Quaternion normalized() const;

    /**
     * @brief インプレース正規化
     */
    void normalize();

    // ========== オイラー角変換 ==========

    /**
     * @brief オイラー角からクォータニオンを生成
     *
     * ZYX順（ヨー、ピッチ、ロール）のオイラー角から変換
     * 航空機の慣例に従います
     *
     * @param roll  ロール角（x軸周り）[rad]
     * @param pitch ピッチ角（y軸周り）[rad]
     * @param yaw   ヨー角（z軸周り）[rad]
     * @return 対応するクォータニオン
     */
    static Quaternion fromEuler(double roll, double pitch, double yaw);

    /**
     * @brief クォータニオンをオイラー角に変換
     *
     * ZYX順（ヨー、ピッチ、ロール）で返します
     *
     * @param roll  出力: ロール角 [rad]
     * @param pitch 出力: ピッチ角 [rad]
     * @param yaw   出力: ヨー角 [rad]
     */
    void toEuler(double& roll, double& pitch, double& yaw) const;

    /**
     * @brief オイラー角を Vector3 として取得
     * @return [roll, pitch, yaw] in radians
     */
    Vector3 toEulerAngles() const;

    // ========== 回転軸・回転角変換 ==========

    /**
     * @brief 回転軸と回転角からクォータニオンを生成
     *
     * 数学的定義:
     * q = [cos(θ/2), sin(θ/2) * axis]
     *
     * @param axis 回転軸（単位ベクトルであること）
     * @param angle 回転角 [rad]
     * @return 対応するクォータニオン
     */
    static Quaternion fromAxisAngle(const Vector3& axis, double angle);

    /**
     * @brief クォータニオンから回転軸と回転角を取得
     *
     * @param axis 出力: 回転軸（単位ベクトル）
     * @param angle 出力: 回転角 [rad]
     */
    void toAxisAngle(Vector3& axis, double& angle) const;

    // ========== 補間 ==========

    /**
     * @brief 球面線形補間（SLERP）
     *
     * 2つの回転間を滑らかに補間します
     * 等角速度で回転します
     *
     * @param q0 開始クォータニオン
     * @param q1 終了クォータニオン
     * @param t 補間パラメータ [0,1]
     * @return 補間されたクォータニオン
     */
    static Quaternion slerp(const Quaternion& q0, const Quaternion& q1, double t);

    // ========== ユーティリティ ==========

    /**
     * @brief 内積
     *
     * 2つのクォータニオンの「近さ」を測る
     * dot(q, -q) は負になる（逆回転）
     */
    double dot(const Quaternion& other) const;

    /**
     * @brief デバッグ用文字列表現
     */
    std::string toString() const;

    /**
     * @brief 標準出力への出力
     */
    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q);

    /**
     * @brief スカラー × クォータニオン
     */
    friend Quaternion operator*(double scalar, const Quaternion& q);

    // ========== 静的メソッド ==========

    /**
     * @brief 単位クォータニオン（回転なし）
     * @return [1, 0, 0, 0]
     */
    static Quaternion identity();

private:
    double data_[4];  ///< クォータニオンの要素 [w, x, y, z]
};

} // namespace math
} // namespace stampfly
