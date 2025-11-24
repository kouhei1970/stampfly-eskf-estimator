#include "stampfly_math/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace stampfly {
namespace math {
namespace utils {

Matrix skewSymmetric(const Vector3& v) {
    // スキュー対称行列の生成
    // [v]× = [  0  -z   y ]
    //        [  z   0  -x ]
    //        [ -y   x   0 ]
    //
    // 性質: [v]× * w = v × w

    Matrix result(3, 3, 0.0);
    result(0, 1) = -v.z();
    result(0, 2) =  v.y();
    result(1, 0) =  v.z();
    result(1, 2) = -v.x();
    result(2, 0) = -v.y();
    result(2, 1) =  v.x();
    return result;
}

Matrix quaternionToRotationMatrix(const Quaternion& q) {
    // クォータニオンから回転行列への変換
    //
    // 公式: R = I + 2w[qv]× + 2[qv]×²
    //
    // または直接計算:
    // R = [ 1-2(y²+z²)   2(xy-wz)     2(xz+wy)   ]
    //     [ 2(xy+wz)     1-2(x²+z²)   2(yz-wx)   ]
    //     [ 2(xz-wy)     2(yz+wx)     1-2(x²+y²) ]

    double w = q.w(), x = q.x(), y = q.y(), z = q.z();

    Matrix R(3, 3);
    R(0, 0) = 1.0 - 2.0*(y*y + z*z);
    R(0, 1) = 2.0*(x*y - w*z);
    R(0, 2) = 2.0*(x*z + w*y);

    R(1, 0) = 2.0*(x*y + w*z);
    R(1, 1) = 1.0 - 2.0*(x*x + z*z);
    R(1, 2) = 2.0*(y*z - w*x);

    R(2, 0) = 2.0*(x*z - w*y);
    R(2, 1) = 2.0*(y*z + w*x);
    R(2, 2) = 1.0 - 2.0*(x*x + y*y);

    return R;
}

Quaternion rotationMatrixToQuaternion(const Matrix& R) {
    // 回転行列からクォータニオンへの変換
    //
    // アルゴリズム: Shepperdの方法
    // 最大の対角要素を選んで数値安定性を確保

    if (R.rows() != 3 || R.cols() != 3) {
        throw std::invalid_argument("rotationMatrixToQuaternion: Matrix must be 3x3");
    }

    double trace = R(0, 0) + R(1, 1) + R(2, 2);
    double w, x, y, z;

    if (trace > 0.0) {
        // w が最大の場合
        double s = 0.5 / std::sqrt(trace + 1.0);
        w = 0.25 / s;
        x = (R(2, 1) - R(1, 2)) * s;
        y = (R(0, 2) - R(2, 0)) * s;
        z = (R(1, 0) - R(0, 1)) * s;
    } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
        // x が最大の場合
        double s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
        w = (R(2, 1) - R(1, 2)) / s;
        x = 0.25 * s;
        y = (R(0, 1) + R(1, 0)) / s;
        z = (R(0, 2) + R(2, 0)) / s;
    } else if (R(1, 1) > R(2, 2)) {
        // y が最大の場合
        double s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
        w = (R(0, 2) - R(2, 0)) / s;
        x = (R(0, 1) + R(1, 0)) / s;
        y = 0.25 * s;
        z = (R(1, 2) + R(2, 1)) / s;
    } else {
        // z が最大の場合
        double s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
        w = (R(1, 0) - R(0, 1)) / s;
        x = (R(0, 2) + R(2, 0)) / s;
        y = (R(1, 2) + R(2, 1)) / s;
        z = 0.25 * s;
    }

    return Quaternion(w, x, y, z).normalized();
}

Quaternion smallAngleQuaternion(const Vector3& delta_theta) {
    // 小さな角度ベクトルからクォータニオンを生成
    //
    // 1次近似: δq ≈ [1, δθ/2]
    //
    // より正確には:
    // δq = [cos(|δθ|/2), sin(|δθ|/2) * δθ/|δθ|]

    double angle = delta_theta.norm();

    if (angle < 1e-8) {
        // 非常に小さい場合は1次近似
        return Quaternion(1.0, delta_theta * 0.5).normalized();
    } else {
        // 正確な計算
        double half_angle = angle * 0.5;
        double s = std::sin(half_angle) / angle;
        return Quaternion(
            std::cos(half_angle),
            delta_theta.x() * s,
            delta_theta.y() * s,
            delta_theta.z() * s
        );
    }
}

Vector3 gravityVector(double g_magnitude) {
    // NED座標系の重力ベクトル
    // 下向き（+Z方向）
    return Vector3(0.0, 0.0, g_magnitude);
}

Vector3 clamp(const Vector3& v, double min_val, double max_val) {
    // 各要素をクランプ
    return Vector3(
        std::max(min_val, std::min(max_val, v.x())),
        std::max(min_val, std::min(max_val, v.y())),
        std::max(min_val, std::min(max_val, v.z()))
    );
}

Vector3 clampNorm(const Vector3& v, double max_norm) {
    // ベクトルのノルムをクランプ
    double norm = v.norm();
    if (norm > max_norm) {
        return v * (max_norm / norm);
    }
    return v;
}

double wrapToPi(double angle) {
    // 角度を [-π, π] に正規化
    // fmod を使用してラップ
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
    return angle - M_PI;
}

double wrapTo2Pi(double angle) {
    // 角度を [0, 2π] に正規化
    angle = std::fmod(angle, 2.0 * M_PI);
    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

double rad2deg(double radians) {
    return radians * 180.0 / M_PI;
}

double mahalanobisDistance(const Matrix& residual, const Matrix& covariance) {
    // マハラノビス距離の2乗を計算
    // d² = r^T * P^(-1) * r
    //
    // ここで:
    // - r: 残差ベクトル (n×1)
    // - P: 共分散行列 (n×n)

    if (residual.cols() != 1) {
        throw std::invalid_argument("mahalanobisDistance: residual must be column vector");
    }
    if (!covariance.isSquare() || covariance.rows() != residual.rows()) {
        throw std::invalid_argument("mahalanobisDistance: incompatible dimensions");
    }

    // P^(-1) * r を計算
    Matrix P_inv = covariance.inverse();
    Matrix P_inv_r = P_inv * residual;

    // r^T * (P^(-1) * r) を計算
    Matrix r_T = residual.transpose();
    Matrix result = r_T * P_inv_r;

    // 結果は 1×1 行列
    return result(0, 0);
}

double numericalDerivative(double (*f)(double), double x0, double h) {
    // 中心差分法
    // f'(x0) ≈ (f(x0 + h) - f(x0 - h)) / (2h)
    //
    // 誤差: O(h²)（前進差分 O(h) より精度が高い）

    double f_plus = f(x0 + h);
    double f_minus = f(x0 - h);
    return (f_plus - f_minus) / (2.0 * h);
}

bool isSymmetric(const Matrix& m, double epsilon) {
    // 対称性判定: A = A^T
    if (!m.isSquare()) {
        return false;
    }

    int n = m.rows();
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (std::abs(m(i, j) - m(j, i)) > epsilon) {
                return false;
            }
        }
    }
    return true;
}

bool isPositiveDefinite(const Matrix& m) {
    // 正定値性の簡易判定
    // 必要条件:
    // 1. 対称行列
    // 2. すべての対角要素が正
    //
    // 注意: これは十分条件ではない
    // 厳密には固有値をすべて計算する必要がある

    if (!isSymmetric(m)) {
        return false;
    }

    int n = m.rows();
    for (int i = 0; i < n; ++i) {
        if (m(i, i) <= 0.0) {
            return false;
        }
    }

    return true;
}

Matrix enforceCovarianceProperties(const Matrix& P, double epsilon) {
    // 共分散行列の修正
    //
    // 1. 対称化: P_sym = (P + P^T) / 2
    // 2. 正定値保証: 対角要素が epsilon 以上であることを保証

    if (!P.isSquare()) {
        throw std::invalid_argument("enforceCovarianceProperties: Matrix must be square");
    }

    // 対称化
    Matrix P_T = P.transpose();
    Matrix P_sym = (P + P_T) * 0.5;

    // 正定値保証（対角要素の最小値を設定）
    int n = P_sym.rows();
    for (int i = 0; i < n; ++i) {
        if (P_sym(i, i) < epsilon) {
            P_sym(i, i) = epsilon;
        }
    }

    return P_sym;
}

} // namespace utils
} // namespace math
} // namespace stampfly
