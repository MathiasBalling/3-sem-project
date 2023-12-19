# Test goertzel
add_executable(goert goertzel/goert.cpp)
target_link_libraries(goert PRIVATE Protocol)
add_test(NAME goertzel COMMAND goert)

# Test the protocol

