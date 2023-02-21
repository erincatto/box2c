cmake_minimum_required(VERSION 3.23)

message(STATUS "cmake tool chain: ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "cmake source dir: ${CMAKE_SOURCE_DIR}")

project(box2d LANGUAGES C CXX)

option(BOX2D_BUILD_SAMPLES "Build the Box2D samples application" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

#if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
#  set(CMAKE_CXX_FLAGS_DEBUG "/ZI")
#  set(CMAKE_SHARED_LINKER_FLAGS "/SAFESEH:NO")
#  set(CMAKE_EXE_LINKER_FLAGS "/SAFESEH:NO")
#endif()

if (MSVC AND WIN32)
    # Enable edit and continue
    add_link_options(/INCREMENTAL)
    add_compile_options(/ZI)
endif()

add_subdirectory(test)
add_subdirectory(src)

if (BOX2D_BUILD_SAMPLES)
	add_subdirectory(samples)

	# default startup project for Visual Studio
	if (MSVC)
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT samples)
		set_property(TARGET samples PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
	endif()	
endif()

# https://stackoverflow.com/questions/72635402/how-to-set-working-directory-in-cmake-visual-studio-2022
# launch.vs.json
# "currentDir": "${workspaceRoot}"