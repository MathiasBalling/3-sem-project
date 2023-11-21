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

#Main implementations
add_executable(Audio
Audio/main.cpp
Audio/PAsound.cpp
Audio/PAsound.h
Audio/goertzel.cpp
Audio/goertzel.h
Audio/protocol.cpp
Audio/consts.h
Audio/SoundObject.h)
target_link_libraries(Audio
-L/opt/homebrew/Cellar/portaudio/19.7.0/lib -lportaudio)

# Tests--------------------------------------------------------------
#To make visualisation of dtmf sounds
add_executable(Visual
Tests/Visual/visual.cpp)
target_link_libraries(Visual ${OpenCV_LIBS})

add_executable(Float Tests/float/float.cpp)
