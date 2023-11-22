cmake_minimum_required(VERSION 3.25.0)
project(3Semester)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_PARALLEL_LEVEL 10)

# Use PkgConfig to find portaudio
find_package(PkgConfig REQUIRED)
#Find portaudio
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
include_directories(${PORTAUDIO_INCLUDE_DIRS})
set(PORTAUDIO_DIRS -L/opt/homebrew/Cellar/portaudio/19.7.0/lib -lportaudio)

# Find opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find wxWidgets
find_package(wxWidgets REQUIRED COMPONENTS core base)
if(wxWidgets_USE_FILE) # not defined in CONFIG mode
  include(${wxWidgets_USE_FILE})
endif()
if(wxWidgets_FOUND)
  message(STATUS "Found wxWidgets LDFLAGS: ${wxWidgets_LIBRARIES}")
else()
  message(STATUS "wxWidgets not found")
endif()



file(GLOB AUDIO_SRC Audio/*.cpp Audio/*.h)
include_directories(Audio/)

file(GLOB COMPUTER_SRC Computer/*.cpp Computer/*.h)

file(GLOB ROBOT_SRC Tests/*.cpp Tests/*.h)

#Main implementations
add_executable(AudioComputer
${COMPUTER_SRC}
${AUDIO_SRC})
target_link_libraries(AudioComputer
${wxWidgets_LIBRARIES}
${PORTAUDIO_DIRS})

# Tests--------------------------------------------------------------
#To make visualisation of dtmf sounds
add_executable(Visual
Tests/Visual/visual.cpp)
target_link_libraries(Visual ${OpenCV_LIBS})

add_executable(Float Tests/float/float.cpp)
