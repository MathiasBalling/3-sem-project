cmake_minimum_required(VERSION 3.25.0)
project(3Semester)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_PARALLEL_LEVEL 10)

enable_testing()

add_subdirectory(Protocol)
add_subdirectory(Computer)
add_subdirectory(Tests)
