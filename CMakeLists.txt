cmake_minimum_required(VERSION 3.25.0)
project(3Semester)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(PkgConfig REQUIRED)
#Find portaudio
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
include_directories(${PORTAUDIO_INCLUDE_DIRS})

#Find opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(Audio/)

file(GLOB AUDIO_SRC Audio/*.cpp Audio/*.h)

#Main implementations
add_executable(AudioComputer
Computer/main.cpp
${AUDIO_SRC})
target_link_libraries(AudioComputer
-L/opt/homebrew/Cellar/portaudio/19.7.0/lib -lportaudio)

# Tests--------------------------------------------------------------
#To make visualisation of dtmf sounds
add_executable(Visual
Tests/Visual/visual.cpp)
target_link_libraries(Visual ${OpenCV_LIBS})

add_executable(Float Tests/float/float.cpp)

add_executable(AudioFix
Tests/Audiofix/audiofix.cpp
${AUDIO_SRC})
target_link_libraries(AudioFix
-L/opt/homebrew/Cellar/portaudio/19.7.0/lib -lportaudio)
