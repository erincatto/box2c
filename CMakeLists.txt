cmake_minimum_required(VERSION 3.23)

message(STATUS "cmake tool chain: ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "cmake source dir: ${CMAKE_SOURCE_DIR}")

project(box2d LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory(src)

# https://stackoverflow.com/questions/72635402/how-to-set-working-directory-in-cmake-visual-studio-2022
# launch.vs.json
# "currentDir": "${workspaceRoot}"