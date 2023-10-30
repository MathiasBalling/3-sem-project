cmake_minimum_required(VERSION 3.25.0)
project(3semester)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
#Find portaudio
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
include_directories(${PORTAUDIO_INCLUDE_DIRS})

#Find opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#To make dtmf sounds
add_executable(Output 
Output/main.cpp
Output/PAsound.cpp
Output/PAsound.h)
target_link_libraries(Output -L/opt/homebrew/Cellar/portaudio/19.7.0/lib -lportaudio)

#To receive dtmf sounds
add_executable(Input
Input/receiver.cpp)
target_link_libraries(Input -L/opt/homebrew/Cellar/portaudio/19.7.0/lib -lportaudio)

#To convert dtmf sounds to data and vice versa
add_executable(Protocol 
Protocol/main.cpp
Protocol/consts.h)

# Tests--------------------------------------------------------------
#To test goertzel algorithm
add_executable(Goertzel
Tests/Goertzel-algorithm/goertzel.cpp)

#To test converting data to protocol
add_executable(DataToProtocol
Tests/Convert-to-protocol/main.cpp)

#To make visualisation of dtmf sounds
add_executable(Visual 
Tests/Visual/visual.cpp)
target_link_libraries(Visual ${OpenCV_LIBS})
