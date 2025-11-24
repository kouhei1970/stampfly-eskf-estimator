# Reference Implementation CMakeLists.txt
# These implementations are for documentation and educational purposes

# Standard EKF reference implementation
add_library(ekf_reference STATIC
    ekf_position_estimator.cpp
)
target_include_directories(ekf_reference PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(ekf_reference PUBLIC Eigen3::Eigen)

# ESKF reference implementation
add_library(eskf_reference STATIC
    eskf_position_estimator.cpp
)
target_include_directories(eskf_reference PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(eskf_reference PUBLIC Eigen3::Eigen)

message(STATUS "  Reference implementations configured")
