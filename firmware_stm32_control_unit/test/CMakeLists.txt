cmake_minimum_required(VERSION 3.15)
project(SigmoidProfileTest CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(sigmoid_test main.cpp)

# Include directories für Header
target_include_directories(sigmoid_test PRIVATE "${CMAKE_SOURCE_DIR}/..")
