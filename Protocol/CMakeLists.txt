project(Protocol)
# Use PkgConfig to find portaudio
find_package(PkgConfig REQUIRED)

#Find portaudio
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

#Add portaudio libraries
set(PORTAUDIO_LIBS
-L${PORTAUDIO_LIBRARY_DIRS}
-lportaudio)

# Find all source files in the protocol directory
file(GLOB SOURCES *.cpp)

# Add the protocol library
add_library(${PROJECT_NAME} STATIC
${SOURCES})

# Add compile options
target_compile_options(${PROJECT_NAME} PRIVATE
-Wall -Wextra -Wpedantic -Wshadow
-Wnon-virtual-dtor -Wcast-align -Wunused
-Woverloaded-virtual -Werror=vla
-Wmisleading-indentation -Wnull-dereference)

# Link portaudio
target_link_libraries(${PROJECT_NAME} PUBLIC
${PORTAUDIO_LIBS})

# Add the include directories when using this library
target_include_directories(${PROJECT_NAME} PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
${PORTAUDIO_INCLUDE_DIRS})
