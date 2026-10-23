cmake_minimum_required(VERSION 3.16)
project(f1_hotlap CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(SDL2 REQUIRED)

add_executable(f1_hotlap src/main.cpp)
target_link_libraries(f1_hotlap PRIVATE SDL2::SDL2)
if(MSVC)
  target_compile_options(f1_hotlap PRIVATE /W4)
  target_compile_definitions(f1_hotlap PRIVATE _USE_MATH_DEFINES)
else()
  target_compile_options(f1_hotlap PRIVATE -Wall -Wextra)
endif()

enable_testing()
add_test(NAME selftest COMMAND f1_hotlap --selftest)
