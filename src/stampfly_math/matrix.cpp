#include "stampfly_math/matrix.hpp"
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace stampfly {
namespace math {

// ========== コンストラクタ ==========

Matrix::Matrix() : rows_(0), cols_(0) {}

Matrix::Matrix(int rows, int cols) : rows_(rows), cols_(cols) {
    if (rows < 0 || cols < 0) {
        throw std::invalid_argument("Matrix: Invalid size");
    }
    data_.resize(rows * cols, 0.0);
}

Matrix::Matrix(int rows, int cols, double value) : rows_(rows), cols_(cols) {
    if (rows < 0 || cols < 0) {
        throw std::invalid_argument("Matrix: Invalid size");
    }
    data_.resize(rows * cols, value);
}

Matrix::Matrix(int rows, int cols, const std::vector<double>& data)
    : rows_(rows), cols_(cols), data_(data) {
    if (rows < 0 || cols < 0) {
        throw std::invalid_argument("Matrix: Invalid size");
    }
    if (static_cast<int>(data.size()) != rows * cols) {
        throw std::invalid_argument("Matrix: Data size mismatch");
    }
}

// ========== サイズ・形状 ==========

void Matrix::resize(int rows, int cols) {
    if (rows < 0 || cols < 0) {
        throw std::invalid_argument("Matrix: Invalid size");
    }
    rows_ = rows;
    cols_ = cols;
    data_.resize(rows * cols, 0.0);
}

// ========== 要素アクセス ==========

double& Matrix::operator()(int i, int j) {
    checkBounds(i, j);
    return data_[i * cols_ + j];
}

double Matrix::operator()(int i, int j) const {
    checkBounds(i, j);
    return data_[i * cols_ + j];
}

void Matrix::set(int i, int j, double value) {
    checkBounds(i, j);
    data_[i * cols_ + j] = value;
}

double Matrix::get(int i, int j) const {
    checkBounds(i, j);
    return data_[i * cols_ + j];
}

// ========== 行列演算 ==========

Matrix Matrix::operator+(const Matrix& other) const {
    // 行列の加算
    // 条件: 同じサイズであること
    checkSizeMatch(other);

    Matrix result(rows_, cols_);
    for (int i = 0; i < rows_ * cols_; ++i) {
        result.data_[i] = data_[i] + other.data_[i];
    }
    return result;
}

Matrix Matrix::operator-(const Matrix& other) const {
    // 行列の減算
    checkSizeMatch(other);

    Matrix result(rows_, cols_);
    for (int i = 0; i < rows_ * cols_; ++i) {
        result.data_[i] = data_[i] - other.data_[i];
    }
    return result;
}

Matrix Matrix::operator*(const Matrix& other) const {
    // 行列の乗算
    // A(m×n) * B(n×p) = C(m×p)
    //
    // 定義: C[i][j] = Σ_{k=0}^{n-1} A[i][k] * B[k][j]
    //
    // 計算量: O(mnp)

    if (cols_ != other.rows_) {
        throw std::invalid_argument("Matrix: Incompatible dimensions for multiplication");
    }

    int m = rows_;
    int n = cols_;
    int p = other.cols_;

    Matrix result(m, p, 0.0);

    // 3重ループで計算
    // i: 結果の行
    // j: 結果の列
    // k: 内積の要素
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < p; ++j) {
            double sum = 0.0;
            for (int k = 0; k < n; ++k) {
                sum += (*this)(i, k) * other(k, j);
            }
            result(i, j) = sum;
        }
    }

    return result;
}

Matrix Matrix::operator*(double scalar) const {
    // スカラー倍
    Matrix result(rows_, cols_);
    for (int i = 0; i < rows_ * cols_; ++i) {
        result.data_[i] = data_[i] * scalar;
    }
    return result;
}

Matrix Matrix::operator/(double scalar) const {
    // スカラー除算
    if (std::abs(scalar) < 1e-15) {
        throw std::runtime_error("Matrix: Division by zero");
    }
    return *this * (1.0 / scalar);
}

Matrix Matrix::operator-() const {
    // 単項マイナス
    Matrix result(rows_, cols_);
    for (int i = 0; i < rows_ * cols_; ++i) {
        result.data_[i] = -data_[i];
    }
    return result;
}

Matrix& Matrix::operator+=(const Matrix& other) {
    checkSizeMatch(other);
    for (int i = 0; i < rows_ * cols_; ++i) {
        data_[i] += other.data_[i];
    }
    return *this;
}

Matrix& Matrix::operator-=(const Matrix& other) {
    checkSizeMatch(other);
    for (int i = 0; i < rows_ * cols_; ++i) {
        data_[i] -= other.data_[i];
    }
    return *this;
}

Matrix& Matrix::operator*=(double scalar) {
    for (int i = 0; i < rows_ * cols_; ++i) {
        data_[i] *= scalar;
    }
    return *this;
}

// ========== 転置・逆行列 ==========

Matrix Matrix::transpose() const {
    // 転置行列
    // A^T[i][j] = A[j][i]
    //
    // m×n 行列の転置は n×m 行列

    Matrix result(cols_, rows_);
    for (int i = 0; i < rows_; ++i) {
        for (int j = 0; j < cols_; ++j) {
            result(j, i) = (*this)(i, j);
        }
    }
    return result;
}

Matrix Matrix::inverse() const {
    // 逆行列の計算
    // アルゴリズム: Gauss-Jordan 消去法
    //
    // 手順:
    // 1. [A | I] の拡大行列を作成
    // 2. 前進消去で [U | B] にする（Uは上三角）
    // 3. 後退代入で [I | A^(-1)] にする
    //
    // 注意:
    // - 正方行列のみ
    // - 特異行列（det=0）は逆行列を持たない

    if (!isSquare()) {
        throw std::runtime_error("Matrix: Cannot invert non-square matrix");
    }

    int n = rows_;

    // 拡大行列 [A | I] を作成
    Matrix augmented(n, 2 * n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            augmented(i, j) = (*this)(i, j);
        }
        // 右半分に単位行列
        augmented(i, i + n) = 1.0;
    }

    // Gauss-Jordan 消去法
    for (int pivot = 0; pivot < n; ++pivot) {
        // ピボット選択（部分ピボット選択）
        int max_row = pivot;
        double max_val = std::abs(augmented(pivot, pivot));
        for (int i = pivot + 1; i < n; ++i) {
            double val = std::abs(augmented(i, pivot));
            if (val > max_val) {
                max_val = val;
                max_row = i;
            }
        }

        // 特異行列チェック
        if (max_val < 1e-10) {
            throw std::runtime_error("Matrix: Singular matrix (not invertible)");
        }

        // 行交換
        if (max_row != pivot) {
            for (int j = 0; j < 2 * n; ++j) {
                std::swap(augmented(pivot, j), augmented(max_row, j));
            }
        }

        // ピボット行を1にスケール
        double pivot_val = augmented(pivot, pivot);
        for (int j = 0; j < 2 * n; ++j) {
            augmented(pivot, j) /= pivot_val;
        }

        // ピボット列の他の要素を0にする
        for (int i = 0; i < n; ++i) {
            if (i != pivot) {
                double factor = augmented(i, pivot);
                for (int j = 0; j < 2 * n; ++j) {
                    augmented(i, j) -= factor * augmented(pivot, j);
                }
            }
        }
    }

    // 右半分が逆行列
    Matrix result(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            result(i, j) = augmented(i, j + n);
        }
    }

    return result;
}

double Matrix::determinant() const {
    // 行列式の計算
    // アルゴリズム: LU分解を利用
    //
    // det(A) = det(L) * det(U) = 1 * Π(U[i][i])
    //
    // 注意: 正方行列のみ

    if (!isSquare()) {
        throw std::runtime_error("Matrix: Determinant only defined for square matrices");
    }

    int n = rows_;

    // 作業用行列（LU分解）
    Matrix work = *this;
    double det = 1.0;

    // ガウス消去法でLU分解
    for (int pivot = 0; pivot < n; ++pivot) {
        // ピボット選択
        int max_row = pivot;
        double max_val = std::abs(work(pivot, pivot));
        for (int i = pivot + 1; i < n; ++i) {
            double val = std::abs(work(i, pivot));
            if (val > max_val) {
                max_val = val;
                max_row = i;
            }
        }

        // 行交換
        if (max_row != pivot) {
            for (int j = pivot; j < n; ++j) {
                std::swap(work(pivot, j), work(max_row, j));
            }
            // 行交換で行列式の符号が変わる
            det = -det;
        }

        // ピボットが0なら行列式は0
        if (std::abs(work(pivot, pivot)) < 1e-10) {
            return 0.0;
        }

        // 前進消去
        for (int i = pivot + 1; i < n; ++i) {
            double factor = work(i, pivot) / work(pivot, pivot);
            for (int j = pivot; j < n; ++j) {
                work(i, j) -= factor * work(pivot, j);
            }
        }

        // 対角要素を掛ける
        det *= work(pivot, pivot);
    }

    return det;
}

// ========== 特殊な行列 ==========

Matrix Matrix::identity(int n) {
    // 単位行列 I[i][j] = δ_ij
    Matrix result(n, n, 0.0);
    for (int i = 0; i < n; ++i) {
        result(i, i) = 1.0;
    }
    return result;
}

Matrix Matrix::zeros(int rows, int cols) {
    return Matrix(rows, cols, 0.0);
}

Matrix Matrix::ones(int rows, int cols) {
    return Matrix(rows, cols, 1.0);
}

Matrix Matrix::diagonal(const std::vector<double>& diag) {
    int n = static_cast<int>(diag.size());
    Matrix result(n, n, 0.0);
    for (int i = 0; i < n; ++i) {
        result(i, i) = diag[i];
    }
    return result;
}

// ========== ブロック操作 ==========

Matrix Matrix::block(int row_start, int row_count, int col_start, int col_count) const {
    // 部分行列の抽出
    if (row_start < 0 || row_start + row_count > rows_ ||
        col_start < 0 || col_start + col_count > cols_) {
        throw std::out_of_range("Matrix: Block out of range");
    }

    Matrix result(row_count, col_count);
    for (int i = 0; i < row_count; ++i) {
        for (int j = 0; j < col_count; ++j) {
            result(i, j) = (*this)(row_start + i, col_start + j);
        }
    }
    return result;
}

void Matrix::setBlock(int row_start, int col_start, const Matrix& block) {
    // 部分行列の設定
    if (row_start < 0 || row_start + block.rows_ > rows_ ||
        col_start < 0 || col_start + block.cols_ > cols_) {
        throw std::out_of_range("Matrix: Block out of range");
    }

    for (int i = 0; i < block.rows_; ++i) {
        for (int j = 0; j < block.cols_; ++j) {
            (*this)(row_start + i, col_start + j) = block(i, j);
        }
    }
}

Matrix Matrix::row(int i) const {
    checkBounds(i, 0);
    return block(i, 1, 0, cols_);
}

Matrix Matrix::col(int j) const {
    checkBounds(0, j);
    return block(0, rows_, j, 1);
}

void Matrix::setRow(int i, const Matrix& row_vector) {
    if (row_vector.rows_ != 1 || row_vector.cols_ != cols_) {
        throw std::invalid_argument("Matrix: Invalid row vector size");
    }
    setBlock(i, 0, row_vector);
}

void Matrix::setCol(int j, const Matrix& col_vector) {
    if (col_vector.rows_ != rows_ || col_vector.cols_ != 1) {
        throw std::invalid_argument("Matrix: Invalid column vector size");
    }
    setBlock(0, j, col_vector);
}

// ========== ユーティリティ ==========

void Matrix::fill(double value) {
    std::fill(data_.begin(), data_.end(), value);
}

void Matrix::setZero() {
    fill(0.0);
}

void Matrix::setIdentity() {
    if (!isSquare()) {
        throw std::runtime_error("Matrix: Identity only defined for square matrices");
    }
    setZero();
    for (int i = 0; i < rows_; ++i) {
        (*this)(i, i) = 1.0;
    }
}

double Matrix::trace() const {
    // トレース: 対角要素の和
    // tr(A) = Σ_i A[i][i]
    if (!isSquare()) {
        throw std::runtime_error("Matrix: Trace only defined for square matrices");
    }
    double sum = 0.0;
    for (int i = 0; i < rows_; ++i) {
        sum += (*this)(i, i);
    }
    return sum;
}

double Matrix::norm() const {
    // フロベニウスノルム
    // ||A||_F = √(Σ_i Σ_j A[i][j]²)
    double sum_sq = 0.0;
    for (const auto& val : data_) {
        sum_sq += val * val;
    }
    return std::sqrt(sum_sq);
}

std::string Matrix::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "[" << rows_ << "x" << cols_ << " matrix]\n";
    for (int i = 0; i < rows_; ++i) {
        oss << "[";
        for (int j = 0; j < cols_; ++j) {
            oss << std::setw(8) << (*this)(i, j);
            if (j < cols_ - 1) oss << ", ";
        }
        oss << "]\n";
    }
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Matrix& m) {
    os << m.toString();
    return os;
}

Matrix operator*(double scalar, const Matrix& m) {
    return m * scalar;
}

// ========== Vector3 との相互運用 ==========

Matrix Matrix::fromVector3(const Vector3& v) {
    Matrix result(3, 1);
    result(0, 0) = v.x();
    result(1, 0) = v.y();
    result(2, 0) = v.z();
    return result;
}

Vector3 Matrix::toVector3() const {
    if (rows_ != 3 || cols_ != 1) {
        throw std::runtime_error("Matrix: Cannot convert to Vector3 (must be 3x1)");
    }
    return Vector3((*this)(0, 0), (*this)(1, 0), (*this)(2, 0));
}

// ========== プライベートヘルパー ==========

void Matrix::checkBounds(int i, int j) const {
    if (i < 0 || i >= rows_ || j < 0 || j >= cols_) {
        throw std::out_of_range("Matrix: Index out of bounds");
    }
}

void Matrix::checkSizeMatch(const Matrix& other) const {
    if (rows_ != other.rows_ || cols_ != other.cols_) {
        throw std::invalid_argument("Matrix: Size mismatch");
    }
}

} // namespace math
} // namespace stampfly
