project(AudioComputer CXX)

# Find wxWidgets
find_package(wxWidgets REQUIRED COMPONENTS core base)
if(wxWidgets_USE_FILE) # not defined in CONFIG mode
  include(${wxWidgets_USE_FILE})
endif()
if(!wxWidgets_FOUND)
  message(STATUS "wxWidgets not found")
endif()

# Add the sources
file(GLOB COMPUTER_SRC *.cpp)

#Add the executable
add_executable(${PROJECT_NAME}
${COMPUTER_SRC})

# Add compile options
target_compile_options(${PROJECT_NAME} PRIVATE
-Wall -Wextra -Wpedantic -Wshadow
-Wnon-virtual-dtor -Wcast-align -Wunused
-Woverloaded-virtual -Werror=vla
-Wmisleading-indentation -Wnull-dereference)

# Link the libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
${wxWidgets_LIBRARIES}
Protocol)

target_include_directories(${PROJECT_NAME} PRIVATE ${wxWidgets_INCLUDE_DIRS})
