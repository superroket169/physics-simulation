cmake_minimum_required(VERSION 3.10)
project(trial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(raylib REQUIRED)
add_executable(${PROJECT_NAME}

    tests/test_torq.cpp
    src/hash.cpp
    src/math.cpp
    src/collision.cpp
    src/math.cpp
    src/obj.cpp
    
)

target_link_libraries(${PROJECT_NAME} raylib GL m pthread dl rt X11)
