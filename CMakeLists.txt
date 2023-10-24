cmake_minimum_required(VERSION 3.25.0)
project(3semester)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
#Find portaudio
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
include_directories(${PORTAUDIO_INCLUDE_DIRS})

add_executable(Protoccol 
Protocol/main.cpp)

add_executable(output 
Output/main.cpp
Output/PAsound.cpp
Output/PAsound.h)

add_executable(input
Input/receiver.cpp)

add_executable(Goertzel
Tests/Goertzel-algorithm/main.cpp)

add_executable(ToProtocol
Tests/Convert-to-protocol/main.cpp)

target_link_libraries(input -L/opt/homebrew/Cellar/portaudio/19.7.0/lib -lportaudio)
target_link_libraries(output -L/opt/homebrew/Cellar/portaudio/19.7.0/lib -lportaudio)




