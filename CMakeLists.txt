cmake_minimum_required(VERSION 3.25.0)
project(3semester)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
#Find portaudio
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
include_directories(${PORTAUDIO_INCLUDE_DIRS})


add_executable(Output 
Output/main.cpp
Output/PAsound.cpp
Output/PAsound.h)
target_link_libraries(Output -L/opt/homebrew/Cellar/portaudio/19.7.0/lib -lportaudio)

add_executable(Input
Input/receiver.cpp)
target_link_libraries(Input -L/opt/homebrew/Cellar/portaudio/19.7.0/lib -lportaudio)

add_executable(Protocol 
Protocol/main.cpp
Protocol/consts.h)

# Tests
add_executable(Goertzel
Tests/Goertzel-algorithm/main.cpp)

add_executable(DataToProtocol
Tests/Convert-to-protocol/main.cpp)




