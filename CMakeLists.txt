cmake_minimum_required(VERSION 3.23)
include(FetchContent)

project(box2d VERSION 3.0.0 LANGUAGES C CXX)

# stuff to help debug cmake
# message(STATUS "cmake source dir: ${CMAKE_SOURCE_DIR}")
# message(STATUS "library postfix: ${CMAKE_DEBUG_POSTFIX}")
# message(STATUS "CMake C compiler: ${CMAKE_C_COMPILER_ID}")
# message(STATUS "CMake C++ compiler: ${CMAKE_CXX_COMPILER_ID}")
# message(STATUS "CMake system name: ${CMAKE_SYSTEM_NAME}")

# This hides samples, test, and doxygen from apps that use box2d via FetchContent
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)

	option(BOX2D_SAMPLES "Build the Box2D samples" ON)
	option(BOX2D_DOCS "Build the Box2D documentation" OFF)
	option(BOX2D_PROFILE "Enable profiling with Tracy" OFF)

	# Hide internal functions
	# todo not sure if this matters in C
	# https://gcc.gnu.org/wiki/Visibility
	# set(CMAKE_C_VISIBILITY_PRESET hidden)
	# set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

	set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	set(CMAKE_VERBOSE_MAKEFILE ON)

	set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
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

	# Used in tests and samples
	FetchContent_Declare(
		enkits
		GIT_REPOSITORY https://github.com/dougbinks/enkiTS.git
		GIT_TAG master
		GIT_SHALLOW TRUE
		GIT_PROGRESS TRUE
	)
	FetchContent_MakeAvailable(enkits)

	add_subdirectory(test)

	if (BOX2D_SAMPLES)
		add_subdirectory(samples)

		# default startup project for Visual Studio
		if (MSVC)
			set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT samples)
			set_property(TARGET samples PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
		endif()	
	endif()

	if (BOX2D_DOCS)
		add_subdirectory(docs)
	endif()

endif()

# The Box2D library uses simde https://github.com/simd-everywhere/simde
add_subdirectory(extern/simde)
add_subdirectory(src)
