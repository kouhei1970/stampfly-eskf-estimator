#pragma once

#include "vector3.hpp"
#include "matrix.hpp"
#include "quaternion.hpp"

namespace stampfly {
namespace math {

/**
 * @brief 数学ユーティリティ関数
 *
 * ESKFや一般的な数値計算で使用される便利な関数群
 */
namespace utils {

/**
 * @brief スキュー対称行列を生成
 *
 * ベクトル v = [x, y, z]^T に対して、外積を行列乗算で表現する行列:
 *
 * [v]× = [  0  -z   y ]
 *        [  z   0  -x ]
 *        [ -y   x   0 ]
 *
 * 性質: [v]× * w = v × w (外積)
 *
 * 用途:
 * - クォータニオン微分方程式
 * - 回転行列の微分
 * - ESKFの状態遷移行列
 *
 * @param v ベクトル
 * @return 3×3 スキュー対称行列
 */
Matrix skewSymmetric(const Vector3& v);

/**
 * @brief クォータニオンから回転行列を生成
 *
 * クォータニオン q を 3×3 回転行列 R に変換
 * R * v は q で v を回転するのと同じ結果
 *
 * 数学的定義:
 * R = I + 2w[qv]× + 2[qv]×²
 *
 * ここで qv = [x, y, z] はクォータニオンのベクトル部
 *
 * @param q クォータニオン（正規化されていること）
 * @return 3×3 回転行列
 */
Matrix quaternionToRotationMatrix(const Quaternion& q);

/**
 * @brief 回転行列からクォータニオンを生成
 *
 * 3×3 回転行列 R をクォータニオンに変換
 *
 * @param R 回転行列（直交行列であること）
 * @return 対応するクォータニオン
 */
Quaternion rotationMatrixToQuaternion(const Matrix& R);

/**
 * @brief 小さな角度ベクトルからクォータニオンを生成
 *
 * ESKF で使用: エラー状態の姿勢誤差（3次元ベクトル）を
 * 小さな回転クォータニオンに変換
 *
 * 近似式（1次近似）:
 * δq ≈ [1, δθ/2]
 *
 * ここで δθ = [δθx, δθy, δθz] は小さな回転ベクトル
 *
 * @param delta_theta 小さな回転ベクトル [rad]
 * @return 対応するクォータニオン（正規化済み）
 */
Quaternion smallAngleQuaternion(const Vector3& delta_theta);

/**
 * @brief 重力ベクトル（NED座標系）
 *
 * NED座標系では下向きが+Z方向なので:
 * g = [0, 0, 9.81]^T
 *
 * @param g_magnitude 重力加速度の大きさ [m/s²]（デフォルト: 9.81）
 * @return 重力ベクトル
 */
Vector3 gravityVector(double g_magnitude = 9.81);

/**
 * @brief ベクトルのクランプ（要素ごと）
 *
 * ベクトルの各要素を [min_val, max_val] の範囲に制限
 *
 * @param v ベクトル
 * @param min_val 最小値
 * @param max_val 最大値
 * @return クランプされたベクトル
 */
Vector3 clamp(const Vector3& v, double min_val, double max_val);

/**
 * @brief ベクトルのノルムをクランプ
 *
 * ベクトルの長さを max_norm 以下に制限
 * 方向は維持
 *
 * @param v ベクトル
 * @param max_norm 最大ノルム
 * @return ノルムがクランプされたベクトル
 */
Vector3 clampNorm(const Vector3& v, double max_norm);

/**
 * @brief 角度を [-π, π] に正規化
 *
 * @param angle 角度 [rad]
 * @return 正規化された角度 [rad]
 */
double wrapToPi(double angle);

/**
 * @brief 角度を [0, 2π] に正規化
 *
 * @param angle 角度 [rad]
 * @return 正規化された角度 [rad]
 */
double wrapTo2Pi(double angle);

/**
 * @brief 度からラジアンへ変換
 *
 * @param degrees 角度 [deg]
 * @return 角度 [rad]
 */
double deg2rad(double degrees);

/**
 * @brief ラジアンから度へ変換
 *
 * @param radians 角度 [rad]
 * @return 角度 [deg]
 */
double rad2deg(double radians);

/**
 * @brief マハラノビス距離を計算
 *
 * マハラノビス距離は、残差と共分散を考慮した「距離」
 * 外れ値検出に使用
 *
 * 定義: d² = r^T * P^(-1) * r
 *
 * ここで:
 * - r: 残差ベクトル（観測値 - 予測値）
 * - P: 共分散行列
 *
 * カイ二乗分布に従うため、閾値で外れ値判定可能
 *
 * @param residual 残差ベクトル (n×1 行列)
 * @param covariance 共分散行列 (n×n 行列)
 * @return マハラノビス距離の2乗
 */
double mahalanobisDistance(const Matrix& residual, const Matrix& covariance);

/**
 * @brief 数値微分（中心差分）
 *
 * f(x) の x = x0 における微分を数値的に計算
 *
 * f'(x0) ≈ (f(x0 + h) - f(x0 - h)) / (2h)
 *
 * @param f 関数
 * @param x0 微分点
 * @param h ステップサイズ（デフォルト: 1e-6）
 * @return f'(x0) の近似値
 */
double numericalDerivative(double (*f)(double), double x0, double h = 1e-6);

/**
 * @brief 対称行列かどうか判定
 *
 * A が対称行列 ⇔ A = A^T
 *
 * @param m 行列
 * @param epsilon 許容誤差
 * @return 対称行列なら true
 */
bool isSymmetric(const Matrix& m, double epsilon = 1e-9);

/**
 * @brief 正定値行列かどうか判定（簡易版）
 *
 * すべての固有値が正 ⇔ 正定値
 * 簡易判定: すべての対角要素が正かつ対称
 *
 * 注意: これは必要条件であり十分条件ではない
 * 厳密な判定には固有値計算が必要
 *
 * @param m 行列
 * @return 正定値の可能性が高ければ true
 */
bool isPositiveDefinite(const Matrix& m);

/**
 * @brief 共分散行列の対称性と正定値性を強制
 *
 * 数値誤差で対称性が崩れた共分散行列を修正
 * - 対称化: P = (P + P^T) / 2
 * - 正定値化: 対角要素に小さな値を加算（必要なら）
 *
 * @param P 共分散行列
 * @param epsilon 正定値保証のための最小対角要素
 * @return 修正された共分散行列
 */
Matrix enforceCovarianceProperties(const Matrix& P, double epsilon = 1e-9);

} // namespace utils
} // namespace math
} // namespace stampfly
