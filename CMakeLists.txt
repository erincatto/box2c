# Box2D project

cmake_minimum_required(VERSION 3.23)
include(FetchContent)

message(STATUS "cmake source dir: ${CMAKE_SOURCE_DIR}")
message(STATUS "library postfix: ${CMAKE_DEBUG_POSTFIX}")

project(box2d LANGUAGES C CXX)

option(BOX2D_BUILD_SAMPLES "Build the Box2D samples application" OFF)
option(BOX2D_PROFILE "Enable profiling with Tracy" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED YES)
set(CMAKE_C_EXTENSIONS YES)
set(CMAKE_CXX_STANDARD 17)

if (MSVC AND WIN32)
    # Enable edit and continue only in debug due to perf hit in release
    # add_link_options($<$<CONFIG:Debug>:/INCREMENTAL>)
    # add_compile_options($<$<CONFIG:Debug>:/ZI>)
	# add_compile_options(/fsanitize=address)
	# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/GS- /Gy /O2 /Oi /Ot")
	# set(CMAKE_CXX_FLAGS_RELEASE "/GS- /Gy /O2 /Oi /Ot")
	# set(CMAKE_C_FLAGS_RELWITHDEBINFO "/GS- /Gy /O2 /Oi /Ot")
	# set(CMAKE_C_FLAGS_RELEASE "/GS- /Gy /O2 /Oi /Ot")
endif()

SET(ENKITS_BUILD_EXAMPLES OFF CACHE BOOL "Build enkiTS examples")

FetchContent_Declare(
	enkits
	GIT_REPOSITORY https://github.com/dougbinks/enkiTS.git
	GIT_TAG master
	GIT_SHALLOW TRUE
	GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(enkits)

if (BOX2D_PROFILE)
	add_compile_definitions(BOX2D_PROFILE)

	FetchContent_Declare(
		tracy
		GIT_REPOSITORY https://github.com/wolfpld/tracy.git
		GIT_TAG master
		GIT_SHALLOW TRUE
		GIT_PROGRESS TRUE
	)
	FetchContent_MakeAvailable(tracy)
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