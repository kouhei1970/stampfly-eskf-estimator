# Examples CMakeLists.txt
# 教育目的のサンプルコード集

message(STATUS "Configuring examples using stampfly_math...")

# ========== 数学ライブラリの基本サンプル ==========

# 01: Vector3 の基本操作
add_executable(01_vector_operations 01_vector_operations.cpp)
target_link_libraries(01_vector_operations stampfly_math)

# 02: Quaternion による回転
add_executable(02_quaternion_rotation 02_quaternion_rotation.cpp)
target_link_libraries(02_quaternion_rotation stampfly_math)

# ========== ESKF Estimator サンプル ==========

# ESKF シンプルデモ
add_executable(simple_estimation_eskf simple_estimation_eskf.cpp)
target_link_libraries(simple_estimation_eskf eskf_estimator stampfly_math)

# ESKF 複合シミュレーション（飛行マニューバ）
add_executable(complex_simulation_eskf complex_simulation_eskf.cpp)
target_link_libraries(complex_simulation_eskf eskf_estimator stampfly_math)

# ========== 今後追加予定 ==========
# 03: Matrix 演算
# 04: 簡単なEKF

message(STATUS "")
message(STATUS "Examples configured:")
message(STATUS "  ✓ 01_vector_operations      - Vector3 基本操作")
message(STATUS "  ✓ 02_quaternion_rotation    - Quaternion 回転")
message(STATUS "  ✓ simple_estimation_eskf    - ESKF推定器デモ")
message(STATUS "  ✓ complex_simulation_eskf   - 複合飛行シミュレーション")
message(STATUS "")
message(STATUS "ビルド後、以下で実行:")
message(STATUS "  ./examples/01_vector_operations")
message(STATUS "  ./examples/02_quaternion_rotation")
message(STATUS "  ./examples/simple_estimation_eskf")
message(STATUS "  ./examples/complex_simulation_eskf")
