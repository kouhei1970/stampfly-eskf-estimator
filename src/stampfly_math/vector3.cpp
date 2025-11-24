#include "stampfly_math/vector3.hpp"
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace stampfly {
namespace math {

// ========== コンストラクタ ==========

Vector3::Vector3() {
    data_[0] = 0.0;
    data_[1] = 0.0;
    data_[2] = 0.0;
}

Vector3::Vector3(double x, double y, double z) {
    data_[0] = x;
    data_[1] = y;
    data_[2] = z;
}

Vector3::Vector3(const double data[3]) {
    data_[0] = data[0];
    data_[1] = data[1];
    data_[2] = data[2];
}

// ========== アクセサ ==========

double& Vector3::operator[](int index) {
    if (index < 0 || index >= 3) {
        throw std::out_of_range("Vector3 index out of range");
    }
    return data_[index];
}

double Vector3::operator[](int index) const {
    if (index < 0 || index >= 3) {
        throw std::out_of_range("Vector3 index out of range");
    }
    return data_[index];
}

void Vector3::set(double x, double y, double z) {
    data_[0] = x;
    data_[1] = y;
    data_[2] = z;
}

// ========== ベクトル演算 ==========

Vector3 Vector3::operator+(const Vector3& other) const {
    // ベクトルの加算: 各成分を個別に加算
    // 結果 = [this.x + other.x, this.y + other.y, this.z + other.z]
    return Vector3(
        data_[0] + other.data_[0],
        data_[1] + other.data_[1],
        data_[2] + other.data_[2]
    );
}

Vector3 Vector3::operator-(const Vector3& other) const {
    // ベクトルの減算: 各成分を個別に減算
    // 結果 = [this.x - other.x, this.y - other.y, this.z - other.z]
    return Vector3(
        data_[0] - other.data_[0],
        data_[1] - other.data_[1],
        data_[2] - other.data_[2]
    );
}

Vector3 Vector3::operator*(double scalar) const {
    // スカラー倍: 各成分にスカラー値を乗算
    // 結果 = [scalar * x, scalar * y, scalar * z]
    return Vector3(
        data_[0] * scalar,
        data_[1] * scalar,
        data_[2] * scalar
    );
}

Vector3 Vector3::operator/(double scalar) const {
    // スカラー除算: 各成分をスカラー値で除算
    // 注意: ゼロ除算チェック
    if (std::abs(scalar) < 1e-15) {
        throw std::runtime_error("Vector3: Division by zero");
    }
    return Vector3(
        data_[0] / scalar,
        data_[1] / scalar,
        data_[2] / scalar
    );
}

Vector3 Vector3::operator-() const {
    // 単項マイナス: 各成分の符号を反転
    // 結果 = [-x, -y, -z]
    return Vector3(-data_[0], -data_[1], -data_[2]);
}

Vector3& Vector3::operator+=(const Vector3& other) {
    // 加算代入: this を更新
    data_[0] += other.data_[0];
    data_[1] += other.data_[1];
    data_[2] += other.data_[2];
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& other) {
    // 減算代入: this を更新
    data_[0] -= other.data_[0];
    data_[1] -= other.data_[1];
    data_[2] -= other.data_[2];
    return *this;
}

Vector3& Vector3::operator*=(double scalar) {
    // スカラー倍代入: this を更新
    data_[0] *= scalar;
    data_[1] *= scalar;
    data_[2] *= scalar;
    return *this;
}

Vector3& Vector3::operator/=(double scalar) {
    // スカラー除算代入: this を更新
    if (std::abs(scalar) < 1e-15) {
        throw std::runtime_error("Vector3: Division by zero");
    }
    data_[0] /= scalar;
    data_[1] /= scalar;
    data_[2] /= scalar;
    return *this;
}

// ========== 内積・外積 ==========

double Vector3::dot(const Vector3& other) const {
    // 内積（ドット積）の計算
    // 定義: a · b = a.x*b.x + a.y*b.y + a.z*b.z
    //
    // 幾何学的意味:
    // - a · b = |a| |b| cos(θ)
    // - θ が 90° なら 0（直交）
    // - θ が 0° なら 最大（平行）
    return data_[0] * other.data_[0] +
           data_[1] * other.data_[1] +
           data_[2] * other.data_[2];
}

Vector3 Vector3::cross(const Vector3& other) const {
    // 外積（クロス積）の計算
    // 定義: a × b = [a.y*b.z - a.z*b.y,
    //                a.z*b.x - a.x*b.z,
    //                a.x*b.y - a.y*b.x]
    //
    // 幾何学的意味:
    // - 結果は a と b の両方に垂直
    // - 大きさは |a × b| = |a| |b| sin(θ)
    // - 右手の法則に従う（右手系）
    //
    // 覚え方:
    // x成分: y*z' - z*y' (y,zの組み合わせ)
    // y成分: z*x' - x*z' (z,xの組み合わせ)
    // z成分: x*y' - y*x' (x,yの組み合わせ)
    return Vector3(
        data_[1] * other.data_[2] - data_[2] * other.data_[1],  // x成分
        data_[2] * other.data_[0] - data_[0] * other.data_[2],  // y成分
        data_[0] * other.data_[1] - data_[1] * other.data_[0]   // z成分
    );
}

// ========== ノルム・正規化 ==========

double Vector3::norm() const {
    // ベクトルの長さ（ユークリッドノルム）
    // 定義: |a| = √(a.x² + a.y² + a.z²)
    //
    // ピタゴラスの定理の3次元版
    return std::sqrt(
        data_[0] * data_[0] +
        data_[1] * data_[1] +
        data_[2] * data_[2]
    );
}

double Vector3::squaredNorm() const {
    // ベクトルの長さの2乗
    // 定義: |a|² = a.x² + a.y² + a.z²
    //
    // 注意: sqrt()を呼ばないため高速
    // 長さの比較には squaredNorm() を使うと効率的
    return data_[0] * data_[0] +
           data_[1] * data_[1] +
           data_[2] * data_[2];
}

Vector3 Vector3::normalized() const {
    // 正規化されたベクトルを返す
    // 定義: â = a / |a|
    //
    // 正規化後のベクトルは:
    // - 長さが1（単位ベクトル）
    // - 方向は元のベクトルと同じ
    double len = norm();
    if (len < 1e-15) {
        // ゼロベクトルは正規化できない
        throw std::runtime_error("Vector3: Cannot normalize zero vector");
    }
    return *this / len;
}

void Vector3::normalize() {
    // インプレース正規化
    // このベクトル自身を単位ベクトルに変更
    double len = norm();
    if (len < 1e-15) {
        throw std::runtime_error("Vector3: Cannot normalize zero vector");
    }
    data_[0] /= len;
    data_[1] /= len;
    data_[2] /= len;
}

bool Vector3::isZero(double epsilon) const {
    // ゼロベクトル判定
    // すべての成分の絶対値が epsilon より小さいか
    return std::abs(data_[0]) < epsilon &&
           std::abs(data_[1]) < epsilon &&
           std::abs(data_[2]) < epsilon;
}

// ========== ユーティリティ ==========

std::string Vector3::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "[" << data_[0] << ", " << data_[1] << ", " << data_[2] << "]";
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Vector3& v) {
    os << v.toString();
    return os;
}

Vector3 operator*(double scalar, const Vector3& v) {
    // スカラー × ベクトル（順序逆）
    // k * v も v * k と同じ結果
    return v * scalar;
}

// ========== 静的メソッド ==========

Vector3 Vector3::zero() {
    return Vector3(0.0, 0.0, 0.0);
}

Vector3 Vector3::unitX() {
    return Vector3(1.0, 0.0, 0.0);
}

Vector3 Vector3::unitY() {
    return Vector3(0.0, 1.0, 0.0);
}

Vector3 Vector3::unitZ() {
    return Vector3(0.0, 0.0, 1.0);
}

} // namespace math
} // namespace stampfly
