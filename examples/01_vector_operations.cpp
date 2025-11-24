/**
 * @file 01_vector_operations.cpp
 * @brief Vector3クラスの基本操作を学ぶ
 *
 * このサンプルでは3次元ベクトルの基本的な演算を実演します。
 * プログラミング初心者でも理解できるよう、詳しくコメントしています。
 */

#include "stampfly_math/vector3.hpp"
#include <iostream>
#include <iomanip>

using namespace stampfly::math;

int main() {
    std::cout << "========================================\n";
    std::cout << "  Vector3 クラス - 基本操作\n";
    std::cout << "========================================\n\n";

    // ========== ベクトルの作成 ==========
    std::cout << "【1】ベクトルの作成\n";
    std::cout << "--------------------\n";

    // 3つの値を指定してベクトルを作成
    Vector3 v1(1.0, 2.0, 3.0);
    Vector3 v2(4.0, 5.0, 6.0);

    std::cout << "v1 = " << v1 << "\n";
    std::cout << "v2 = " << v2 << "\n\n";

    // ========== 基本演算 ==========
    std::cout << "【2】基本演算\n";
    std::cout << "--------------------\n";

    // 加算: 各成分を足し算
    Vector3 sum = v1 + v2;
    std::cout << "v1 + v2 = " << sum << "\n";

    // 減算: 各成分を引き算
    Vector3 diff = v1 - v2;
    std::cout << "v1 - v2 = " << diff << "\n";

    // スカラー倍: すべての成分に同じ値を掛ける
    Vector3 scaled = v1 * 2.0;
    std::cout << "v1 * 2.0 = " << scaled << "\n";

    // スカラー除算
    Vector3 divided = v2 / 2.0;
    std::cout << "v2 / 2.0 = " << divided << "\n\n";

    // ========== 内積（ドット積） ==========
    std::cout << "【3】内積（ドット積）\n";
    std::cout << "--------------------\n";
    std::cout << "内積は2つのベクトルの「似ている度合い」を表します\n";
    std::cout << "  - 平行なら最大\n";
    std::cout << "  - 垂直なら0\n";
    std::cout << "  - 逆向きなら負\n\n";

    double dot_product = v1.dot(v2);
    std::cout << "v1 · v2 = " << dot_product << "\n";

    // 垂直なベクトルの例
    Vector3 vx(1.0, 0.0, 0.0);  // X軸方向
    Vector3 vy(0.0, 1.0, 0.0);  // Y軸方向
    std::cout << "\n垂直な例:\n";
    std::cout << "vx = " << vx << "\n";
    std::cout << "vy = " << vy << "\n";
    std::cout << "vx · vy = " << vx.dot(vy) << " (= 0, 垂直)\n\n";

    // ========== 外積（クロス積） ==========
    std::cout << "【4】外積（クロス積）\n";
    std::cout << "--------------------\n";
    std::cout << "外積は2つのベクトルに垂直な新しいベクトルを作ります\n";
    std::cout << "右手の法則に従います\n\n";

    Vector3 cross_product = vx.cross(vy);
    std::cout << "vx × vy = " << cross_product << "\n";
    std::cout << "これはZ軸方向を指します（右手系）\n\n";

    // 確認: 外積の結果は元のベクトルと垂直
    std::cout << "確認: 外積の結果は元のベクトルと垂直\n";
    std::cout << "(vx × vy) · vx = " << cross_product.dot(vx) << " (≈ 0)\n";
    std::cout << "(vx × vy) · vy = " << cross_product.dot(vy) << " (≈ 0)\n\n";

    // ========== ノルム（長さ） ==========
    std::cout << "【5】ノルム（長さ）\n";
    std::cout << "--------------------\n";
    std::cout << "ベクトルの長さを計算します\n\n";

    Vector3 v3(3.0, 4.0, 0.0);
    double length = v3.norm();
    std::cout << "v3 = " << v3 << "\n";
    std::cout << "|v3| = " << length << "\n";
    std::cout << "確認: 3-4-5の直角三角形 → √(3² + 4²) = 5.0\n\n";

    // ========== 正規化 ==========
    std::cout << "【6】正規化（単位ベクトル化）\n";
    std::cout << "--------------------\n";
    std::cout << "方向は同じで、長さを1にします\n\n";

    Vector3 normalized = v3.normalized();
    std::cout << "v3の正規化 = " << normalized << "\n";
    std::cout << "長さ = " << normalized.norm() << " (= 1.0)\n\n";

    // ========== 実用例：2点間の距離 ==========
    std::cout << "【7】実用例：2点間の距離\n";
    std::cout << "--------------------\n";

    Vector3 point1(1.0, 2.0, 3.0);
    Vector3 point2(4.0, 6.0, 8.0);

    Vector3 displacement = point2 - point1;
    double distance = displacement.norm();

    std::cout << "点1: " << point1 << "\n";
    std::cout << "点2: " << point2 << "\n";
    std::cout << "変位ベクトル: " << displacement << "\n";
    std::cout << "距離: " << distance << " m\n\n";

    // ========== 実用例：速度ベクトル ==========
    std::cout << "【8】実用例：速度ベクトル\n";
    std::cout << "--------------------\n";

    Vector3 velocity(10.0, 0.0, -2.0);  // 前進10m/s, 下降2m/s
    double speed = velocity.norm();

    std::cout << "速度ベクトル: " << velocity << " m/s\n";
    std::cout << "速さ: " << std::fixed << std::setprecision(2)
              << speed << " m/s\n";

    Vector3 direction = velocity.normalized();
    std::cout << "進行方向: " << direction << " (単位ベクトル)\n\n";

    // ========== まとめ ==========
    std::cout << "========================================\n";
    std::cout << "  まとめ\n";
    std::cout << "========================================\n";
    std::cout << "✓ ベクトルの加算・減算・スカラー倍\n";
    std::cout << "✓ 内積（ドット積）: 類似度\n";
    std::cout << "✓ 外積（クロス積）: 垂直ベクトル\n";
    std::cout << "✓ ノルム（長さ）\n";
    std::cout << "✓ 正規化（単位ベクトル化）\n";
    std::cout << "\nこれらはロボット制御で頻繁に使います！\n";
    std::cout << "========================================\n";

    return 0;
}
