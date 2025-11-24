#include "stampfly_math/quaternion.hpp"
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <cmath>

namespace stampfly {
namespace math {

// ========== コンストラクタ ==========

Quaternion::Quaternion() {
    // 単位クォータニオン（回転なし）
    data_[0] = 1.0;  // w
    data_[1] = 0.0;  // x
    data_[2] = 0.0;  // y
    data_[3] = 0.0;  // z
}

Quaternion::Quaternion(double w, double x, double y, double z) {
    data_[0] = w;
    data_[1] = x;
    data_[2] = y;
    data_[3] = z;
}

Quaternion::Quaternion(double w, const Vector3& v) {
    data_[0] = w;
    data_[1] = v.x();
    data_[2] = v.y();
    data_[3] = v.z();
}

Quaternion::Quaternion(const double data[4]) {
    data_[0] = data[0];
    data_[1] = data[1];
    data_[2] = data[2];
    data_[3] = data[3];
}

// ========== アクセサ ==========

double& Quaternion::operator[](int index) {
    if (index < 0 || index >= 4) {
        throw std::out_of_range("Quaternion index out of range");
    }
    return data_[index];
}

double Quaternion::operator[](int index) const {
    if (index < 0 || index >= 4) {
        throw std::out_of_range("Quaternion index out of range");
    }
    return data_[index];
}

void Quaternion::set(double w, double x, double y, double z) {
    data_[0] = w;
    data_[1] = x;
    data_[2] = y;
    data_[3] = z;
}

// ========== クォータニオン演算 ==========

Quaternion Quaternion::operator*(const Quaternion& other) const {
    // クォータニオンの乗算
    // q1 * q2 = [w1*w2 - v1·v2, w1*v2 + w2*v1 + v1×v2]
    //
    // 展開すると（Hamilton積）:
    // w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    // x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    // y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    // z = w1*z2 + x1*y2 - y1*x2 + z1*w2

    double w1 = data_[0], x1 = data_[1], y1 = data_[2], z1 = data_[3];
    double w2 = other.data_[0], x2 = other.data_[1], y2 = other.data_[2], z2 = other.data_[3];

    return Quaternion(
        w1*w2 - x1*x2 - y1*y2 - z1*z2,  // w成分（スカラー部）
        w1*x2 + x1*w2 + y1*z2 - z1*y2,  // x成分
        w1*y2 - x1*z2 + y1*w2 + z1*x2,  // y成分
        w1*z2 + x1*y2 - y1*x2 + z1*w2   // z成分
    );
}

Vector3 Quaternion::rotate(const Vector3& v) const {
    // ベクトルの回転: v' = q * v * q^(-1)
    //
    // 効率的な計算式（Rodriguesの回転公式の変形）:
    // v' = v + 2*w*(qv×v) + 2*(qv×(qv×v))
    // ここで qv = [x, y, z] はクォータニオンのベクトル部
    //
    // これは以下より高速:
    // 1. v を純虚数クォータニオン [0, v] に変換
    // 2. q * [0, v] * q^(-1) を計算
    // 3. 結果のベクトル部を抽出

    Vector3 qv(data_[1], data_[2], data_[3]);  // クォータニオンのベクトル部
    double qw = data_[0];                       // クォータニオンのスカラー部

    // 2 * (qv × v)
    Vector3 cross1 = qv.cross(v);
    Vector3 term1 = 2.0 * qw * cross1;

    // 2 * (qv × (qv × v))
    Vector3 cross2 = qv.cross(cross1);
    Vector3 term2 = 2.0 * cross2;

    // v' = v + term1 + term2
    return v + term1 + term2;
}

Quaternion Quaternion::operator*(double scalar) const {
    return Quaternion(
        data_[0] * scalar,
        data_[1] * scalar,
        data_[2] * scalar,
        data_[3] * scalar
    );
}

Quaternion Quaternion::operator/(double scalar) const {
    if (std::abs(scalar) < 1e-15) {
        throw std::runtime_error("Quaternion: Division by zero");
    }
    return Quaternion(
        data_[0] / scalar,
        data_[1] / scalar,
        data_[2] / scalar,
        data_[3] / scalar
    );
}

Quaternion Quaternion::operator+(const Quaternion& other) const {
    return Quaternion(
        data_[0] + other.data_[0],
        data_[1] + other.data_[1],
        data_[2] + other.data_[2],
        data_[3] + other.data_[3]
    );
}

Quaternion Quaternion::operator-(const Quaternion& other) const {
    return Quaternion(
        data_[0] - other.data_[0],
        data_[1] - other.data_[1],
        data_[2] - other.data_[2],
        data_[3] - other.data_[3]
    );
}

// ========== 共役・逆元 ==========

Quaternion Quaternion::conjugate() const {
    // 共役: q* = [w, -v]
    // ベクトル部の符号を反転
    return Quaternion(data_[0], -data_[1], -data_[2], -data_[3]);
}

Quaternion Quaternion::inverse() const {
    // 逆元: q^(-1) = q* / |q|²
    //
    // 正規化クォータニオン（|q|=1）の場合:
    // q^(-1) = q*（共役と等しい）
    double sqNorm = squaredNorm();
    if (sqNorm < 1e-15) {
        throw std::runtime_error("Quaternion: Cannot invert zero quaternion");
    }
    return conjugate() / sqNorm;
}

// ========== ノルム・正規化 ==========

double Quaternion::norm() const {
    return std::sqrt(data_[0]*data_[0] + data_[1]*data_[1] +
                    data_[2]*data_[2] + data_[3]*data_[3]);
}

double Quaternion::squaredNorm() const {
    return data_[0]*data_[0] + data_[1]*data_[1] +
           data_[2]*data_[2] + data_[3]*data_[3];
}

Quaternion Quaternion::normalized() const {
    double len = norm();
    if (len < 1e-15) {
        throw std::runtime_error("Quaternion: Cannot normalize zero quaternion");
    }
    return *this / len;
}

void Quaternion::normalize() {
    double len = norm();
    if (len < 1e-15) {
        throw std::runtime_error("Quaternion: Cannot normalize zero quaternion");
    }
    data_[0] /= len;
    data_[1] /= len;
    data_[2] /= len;
    data_[3] /= len;
}

// ========== オイラー角変換 ==========

Quaternion Quaternion::fromEuler(double roll, double pitch, double yaw) {
    // オイラー角からクォータニオンへの変換
    // ZYX順（ヨー、ピッチ、ロール）
    //
    // 各軸周りの回転クォータニオン:
    // Qz(yaw)   = [cos(yaw/2),   0, 0, sin(yaw/2)]
    // Qy(pitch) = [cos(pitch/2), 0, sin(pitch/2), 0]
    // Qx(roll)  = [cos(roll/2),  sin(roll/2), 0, 0]
    //
    // 合成: Q = Qz * Qy * Qx

    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    return Quaternion(
        cr*cp*cy + sr*sp*sy,  // w
        sr*cp*cy - cr*sp*sy,  // x
        cr*sp*cy + sr*cp*sy,  // y
        cr*cp*sy - sr*sp*cy   // z
    );
}

void Quaternion::toEuler(double& roll, double& pitch, double& yaw) const {
    // クォータニオンからオイラー角への変換
    // ZYX順（ヨー、ピッチ、ロール）
    //
    // 注意: ジンバルロック付近（pitch = ±90°）では精度が低下

    double w = data_[0], x = data_[1], y = data_[2], z = data_[3];

    // Roll (x軸周り)
    double sinr_cosp = 2.0 * (w*x + y*z);
    double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y軸周り)
    double sinp = 2.0 * (w*y - z*x);
    // ジンバルロック対策
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);  // ±90度
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (z軸周り)
    double siny_cosp = 2.0 * (w*z + x*y);
    double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

Vector3 Quaternion::toEulerAngles() const {
    double roll, pitch, yaw;
    toEuler(roll, pitch, yaw);
    return Vector3(roll, pitch, yaw);
}

// ========== 回転軸・回転角変換 ==========

Quaternion Quaternion::fromAxisAngle(const Vector3& axis, double angle) {
    // 回転軸と回転角からクォータニオンを生成
    // q = [cos(θ/2), sin(θ/2) * axis]
    //
    // axis は単位ベクトルであること

    double half_angle = angle * 0.5;
    double s = std::sin(half_angle);
    double c = std::cos(half_angle);

    // axisが正規化されているか確認
    Vector3 normalized_axis = axis;
    if (std::abs(axis.norm() - 1.0) > 1e-6) {
        // 正規化されていない場合は正規化
        normalized_axis = axis.normalized();
    }

    return Quaternion(
        c,                          // w = cos(θ/2)
        s * normalized_axis.x(),    // x = sin(θ/2) * axis.x
        s * normalized_axis.y(),    // y = sin(θ/2) * axis.y
        s * normalized_axis.z()     // z = sin(θ/2) * axis.z
    );
}

void Quaternion::toAxisAngle(Vector3& axis, double& angle) const {
    // クォータニオンから回転軸と回転角を抽出
    //
    // q = [cos(θ/2), sin(θ/2) * axis]
    // より:
    // θ = 2 * acos(w)
    // axis = v / sin(θ/2)

    // まず正規化を確認
    Quaternion q = *this;
    double len = q.norm();
    if (std::abs(len - 1.0) > 1e-6) {
        q.normalize();
    }

    // 回転角を計算
    double w = q.data_[0];
    // w を [-1, 1] にクランプ（数値誤差対策）
    if (w > 1.0) w = 1.0;
    if (w < -1.0) w = -1.0;

    angle = 2.0 * std::acos(w);

    // 回転軸を計算
    double s = std::sin(angle * 0.5);
    if (std::abs(s) < 1e-10) {
        // 回転角がほぼ0の場合（回転なし）
        axis = Vector3::unitZ();  // 任意の軸
        angle = 0.0;
    } else {
        axis = Vector3(q.data_[1] / s, q.data_[2] / s, q.data_[3] / s);
    }
}

// ========== 補間 ==========

Quaternion Quaternion::slerp(const Quaternion& q0, const Quaternion& q1, double t) {
    // 球面線形補間（Spherical Linear Interpolation）
    //
    // 2つの回転間を等角速度で補間
    // 最短経路で回転
    //
    // 数学的定義:
    // slerp(q0, q1, t) = (sin((1-t)θ)/sin(θ))*q0 + (sin(tθ)/sin(θ))*q1
    // ここで θ = acos(q0·q1)

    // t を [0, 1] にクランプ
    if (t <= 0.0) return q0;
    if (t >= 1.0) return q1;

    // 内積を計算
    double dot_product = q0.dot(q1);

    // 最短経路を選択するため、内積が負なら q1 を反転
    Quaternion q1_corrected = q1;
    if (dot_product < 0.0) {
        q1_corrected = q1 * (-1.0);
        dot_product = -dot_product;
    }

    // 内積を [-1, 1] にクランプ
    if (dot_product > 1.0) dot_product = 1.0;

    // ほぼ同じ回転の場合は線形補間（LERP）
    const double threshold = 0.9995;
    if (dot_product > threshold) {
        // LERP + 正規化
        Quaternion result = q0 * (1.0 - t) + q1_corrected * t;
        result.normalize();
        return result;
    }

    // SLERP
    double theta = std::acos(dot_product);
    double sin_theta = std::sin(theta);

    double w0 = std::sin((1.0 - t) * theta) / sin_theta;
    double w1 = std::sin(t * theta) / sin_theta;

    return q0 * w0 + q1_corrected * w1;
}

// ========== ユーティリティ ==========

double Quaternion::dot(const Quaternion& other) const {
    return data_[0]*other.data_[0] + data_[1]*other.data_[1] +
           data_[2]*other.data_[2] + data_[3]*other.data_[3];
}

std::string Quaternion::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "[w:" << data_[0] << ", x:" << data_[1]
        << ", y:" << data_[2] << ", z:" << data_[3] << "]";
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    os << q.toString();
    return os;
}

Quaternion operator*(double scalar, const Quaternion& q) {
    return q * scalar;
}

// ========== 静的メソッド ==========

Quaternion Quaternion::identity() {
    return Quaternion(1.0, 0.0, 0.0, 0.0);
}

} // namespace math
} // namespace stampfly
