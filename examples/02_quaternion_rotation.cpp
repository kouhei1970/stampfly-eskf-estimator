/**
 * @file 02_quaternion_rotation.cpp
 * @brief クォータニオンによる回転を学ぶ
 *
 * クォータニオンは3次元回転を表現する強力な道具です。
 * このサンプルでは基本的な使い方を実演します。
 */

#include "stampfly_math/quaternion.hpp"
#include "stampfly_math/vector3.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace stampfly::math;

int main() {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "========================================\n";
    std::cout << "  Quaternion クラス - 回転の表現\n";
    std::cout << "========================================\n\n";

    // ========== クォータニオンとは？ ==========
    std::cout << "【1】クォータニオンとは？\n";
    std::cout << "--------------------\n";
    std::cout << "クォータニオンは4つの数 [w, x, y, z] で回転を表します\n";
    std::cout << "  w: スカラー部（実部）\n";
    std::cout << "  x, y, z: ベクトル部（虚部）\n\n";

    // 単位クォータニオン（回転なし）
    Quaternion q_identity = Quaternion::identity();
    std::cout << "単位クォータニオン（回転なし）:\n";
    std::cout << "q = " << q_identity << "\n\n";

    // ========== 回転軸と回転角から作成 ==========
    std::cout << "【2】回転軸と回転角から作成\n";
    std::cout << "--------------------\n";

    // Z軸周りに90度回転
    Vector3 axis_z = Vector3::unitZ();  // Z軸 = [0, 0, 1]
    double angle = M_PI / 2.0;          // 90度 = π/2 rad

    Quaternion q_z90 = Quaternion::fromAxisAngle(axis_z, angle);

    std::cout << "回転軸: " << axis_z << " (Z軸)\n";
    std::cout << "回転角: " << angle << " rad (= 90度)\n";
    std::cout << "クォータニオン: " << q_z90 << "\n\n";

    // ========== ベクトルの回転 ==========
    std::cout << "【3】ベクトルの回転\n";
    std::cout << "--------------------\n";

    // X軸方向のベクトル [1, 0, 0]
    Vector3 v_x = Vector3::unitX();
    std::cout << "元のベクトル: " << v_x << " (X軸方向)\n";

    // Z軸周りに90度回転すると、Y軸方向になるはず
    Vector3 v_rotated = q_z90.rotate(v_x);
    std::cout << "Z軸周りに90度回転後: " << v_rotated << "\n";
    std::cout << "→ Y軸方向 [0, 1, 0] になりました！\n\n";

    // ========== オイラー角との相互変換 ==========
    std::cout << "【4】オイラー角との相互変換\n";
    std::cout << "--------------------\n";
    std::cout << "オイラー角（roll, pitch, yaw）との相互変換が可能です\n\n";

    // オイラー角から作成
    double roll = 0.1;    // ロール（X軸周り）
    double pitch = 0.2;   // ピッチ（Y軸周り）
    double yaw = 0.3;     // ヨー（Z軸周り）

    Quaternion q_from_euler = Quaternion::fromEuler(roll, pitch, yaw);
    std::cout << "オイラー角:\n";
    std::cout << "  roll  = " << roll << " rad\n";
    std::cout << "  pitch = " << pitch << " rad\n";
    std::cout << "  yaw   = " << yaw << " rad\n";
    std::cout << "クォータニオン: " << q_from_euler << "\n\n";

    // クォータニオンからオイラー角に戻す
    double roll_back, pitch_back, yaw_back;
    q_from_euler.toEuler(roll_back, pitch_back, yaw_back);
    std::cout << "逆変換:\n";
    std::cout << "  roll  = " << roll_back << " rad\n";
    std::cout << "  pitch = " << pitch_back << " rad\n";
    std::cout << "  yaw   = " << yaw_back << " rad\n";
    std::cout << "→ 元の値に戻りました！\n\n";

    // ========== 回転の合成 ==========
    std::cout << "【5】回転の合成\n";
    std::cout << "--------------------\n";
    std::cout << "クォータニオンの乗算で回転を合成できます\n\n";

    // X軸周りに30度
    Quaternion q_x30 = Quaternion::fromAxisAngle(Vector3::unitX(), M_PI / 6.0);
    // Y軸周りに45度
    Quaternion q_y45 = Quaternion::fromAxisAngle(Vector3::unitY(), M_PI / 4.0);

    // 合成：まずX軸周り30度、次にY軸周り45度
    Quaternion q_combined = q_y45 * q_x30;

    std::cout << "X軸周り30度回転: " << q_x30 << "\n";
    std::cout << "Y軸周り45度回転: " << q_y45 << "\n";
    std::cout << "合成回転: " << q_combined << "\n\n";

    // ベクトルに適用
    Vector3 v_test(1.0, 0.0, 0.0);
    Vector3 v_result = q_combined.rotate(v_test);
    std::cout << "テストベクトル: " << v_test << "\n";
    std::cout << "合成回転後: " << v_result << "\n\n";

    // ========== 逆回転 ==========
    std::cout << "【6】逆回転\n";
    std::cout << "--------------------\n";

    // 回転を元に戻す
    Quaternion q_inverse = q_z90.inverse();
    Vector3 v_back = q_inverse.rotate(v_rotated);

    std::cout << "回転したベクトル: " << v_rotated << "\n";
    std::cout << "逆回転適用: " << v_back << "\n";
    std::cout << "→ 元のベクトルに戻りました！\n\n";

    // ========== 球面線形補間（SLERP） ==========
    std::cout << "【7】球面線形補間（SLERP）\n";
    std::cout << "--------------------\n";
    std::cout << "2つの回転を滑らかに補間します\n\n";

    Quaternion q_start = Quaternion::identity();  // 回転なし
    Quaternion q_end = q_z90;                      // 90度回転

    std::cout << "開始: " << q_start << "\n";
    std::cout << "終了: " << q_end << "\n\n";

    std::cout << "補間（t = 0.0 → 1.0）:\n";
    for (double t = 0.0; t <= 1.0; t += 0.25) {
        Quaternion q_interp = Quaternion::slerp(q_start, q_end, t);
        Vector3 v_interp = q_interp.rotate(v_x);

        std::cout << "  t = " << t << ": ";
        std::cout << "v = " << v_interp << "\n";
    }
    std::cout << "→ X軸方向からY軸方向へ滑らかに回転\n\n";

    // ========== 実用例：ドローンの姿勢 ==========
    std::cout << "【8】実用例：ドローンの姿勢\n";
    std::cout << "--------------------\n";

    // ドローンが少し傾いている姿勢
    double drone_roll = 0.1;   // 5.7度 右に傾斜
    double drone_pitch = -0.05; // 2.9度 機首下げ
    double drone_yaw = M_PI / 4; // 45度 北東方向

    Quaternion drone_attitude = Quaternion::fromEuler(
        drone_roll, drone_pitch, drone_yaw
    );

    std::cout << "ドローンの姿勢:\n";
    std::cout << "  roll  = " << drone_roll * 180.0 / M_PI << "度\n";
    std::cout << "  pitch = " << drone_pitch * 180.0 / M_PI << "度\n";
    std::cout << "  yaw   = " << drone_yaw * 180.0 / M_PI << "度\n";
    std::cout << "クォータニオン表現: " << drone_attitude << "\n\n";

    // ボディ座標系の前方ベクトルがワールド座標系でどこを向いているか
    Vector3 body_forward(1.0, 0.0, 0.0);
    Vector3 world_forward = drone_attitude.rotate(body_forward);
    std::cout << "ボディ座標の前方: " << body_forward << "\n";
    std::cout << "ワールド座標での方向: " << world_forward << "\n\n";

    // ========== まとめ ==========
    std::cout << "========================================\n";
    std::cout << "  まとめ\n";
    std::cout << "========================================\n";
    std::cout << "✓ クォータニオンは4つの数で回転を表現\n";
    std::cout << "✓ 回転軸と回転角から作成可能\n";
    std::cout << "✓ ベクトルを回転できる\n";
    std::cout << "✓ オイラー角と相互変換可能\n";
    std::cout << "✓ 乗算で回転を合成できる\n";
    std::cout << "✓ SLERP で滑らかに補間できる\n";
    std::cout << "\nクォータニオンは姿勢推定の核心です！\n";
    std::cout << "========================================\n";

    return 0;
}
