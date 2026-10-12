cmake_minimum_required(VERSION 3.16)
project(flight_control_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- simulation executable ----
add_executable(sim
    src/app/main.cpp
    src/sim/PhysicsModel.cpp
    src/domain/PIDController.cpp
    src/domain/AltitudeController.cpp
)
target_include_directories(sim PRIVATE src)

# ---- tests ----
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.18.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_executable(tests
    tests/test_pid.cpp
    src/domain/PIDController.cpp
)
target_include_directories(tests PRIVATE src)
target_link_libraries(tests PRIVATE GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(tests)