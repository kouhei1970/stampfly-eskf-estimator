# Tests CMakeLists.txt

message(STATUS "Configuring tests...")

# Unit tests
add_executable(test_eskf_estimator
    unit/test_eskf_estimator.cpp
)

target_link_libraries(test_eskf_estimator
    eskf_estimator
    stampfly_math
)

# Add test to CTest
add_test(NAME ESKFEstimatorTests COMMAND test_eskf_estimator)

message(STATUS "")
message(STATUS "Tests configured:")
message(STATUS "  ✓ test_eskf_estimator - ESKF unit tests")
message(STATUS "")
message(STATUS "Run tests with: ctest or ./tests/test_eskf_estimator")
