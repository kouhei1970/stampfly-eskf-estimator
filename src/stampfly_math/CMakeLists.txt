# StampFly Math Library
# 教育目的の独自実装数学ライブラリ

message(STATUS "Configuring StampFly Math Library...")

# ソースファイルリスト
set(STAMPFLY_MATH_SOURCES
    vector3.cpp
    quaternion.cpp
    matrix.cpp
    math_utils.cpp
)

# 静的ライブラリとしてビルド
add_library(stampfly_math STATIC ${STAMPFLY_MATH_SOURCES})

# インクルードディレクトリを設定
target_include_directories(stampfly_math
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# C++14標準を使用
target_compile_features(stampfly_math PUBLIC cxx_std_14)

# コンパイルオプション
if(MSVC)
    target_compile_options(stampfly_math PRIVATE /W4)
else()
    target_compile_options(stampfly_math PRIVATE
        -Wall -Wextra -Wpedantic
        # 教育目的なので警告を厳しく
    )
endif()

message(STATUS "StampFly Math Library configured successfully")
message(STATUS "  - Vector3: 3D vector operations")
message(STATUS "  - Quaternion: Rotation representation")
message(STATUS "  - Matrix: General-purpose matrix operations")
message(STATUS "  - Math Utils: ESKF-specific utilities")
