# ESKF Estimator Library
# Educational/Research implementation using stampfly_math

add_library(eskf_estimator STATIC
    eskf_estimator.cpp
    filter_parameters.cpp
)

target_include_directories(eskf_estimator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

target_link_libraries(eskf_estimator PUBLIC
    stampfly_math
)

# Set C++14 standard
target_compile_features(eskf_estimator PUBLIC cxx_std_14)

message(STATUS "  ✓ ESKF Estimator (using stampfly_math)")
