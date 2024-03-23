set(BOX2D_SOURCE_FILES
	aabb.c
	aabb.h
	allocate.c
	allocate.h
	array.c
	array.h
	bitset.c
	bitset.h
	block_allocator.c
	block_allocator.h
	block_array.c
	block_array.h
	body.c
	body.h
	broad_phase.c
	broad_phase.h
	constraint_graph.c
	constraint_graph.h
	contact.c
	contact.h
	contact_solver.c
	contact_solver.h
	core.c
	core.h
	ctz.h
	distance.c
	distance_joint.c
	dynamic_tree.c
	geometry.c
	hull.c
	id_pool.c
	id_pool.h
	implementation.c
	island.c
	island.h
	joint.c
	joint.h
	manifold.c
	math.c
	motor_joint.c
	mouse_joint.c
	polygon_shape.h
	pool.c
	pool.h
	prismatic_joint.c
	revolute_joint.c
	shape.c
	shape.h
	solver.c
	solver.h
	solver_set.c
	solver_set.h
	stack_allocator.c
	stack_allocator.h
	static_body.c
	static_body.h
	table.c
	table.h
	timer.c
	types.c
	user_constants.h.in
	util.h
	weld_joint.c
	wheel_joint.c
	world.c
	world.h
)

set(BOX2D_API_FILES
	../include/box2d/api.h
	../include/box2d/box2d.h
	../include/box2d/callbacks.h
	../include/box2d/color.h
	../include/box2d/constants.h
	../include/box2d/debug_draw.h
	../include/box2d/distance.h
	../include/box2d/dynamic_tree.h
	../include/box2d/event_types.h
	../include/box2d/geometry.h
	../include/box2d/hull.h
	../include/box2d/id.h
	../include/box2d/joint_types.h
	../include/box2d/manifold.h
	../include/box2d/math.h
	../include/box2d/math_cpp.h
	../include/box2d/math_types.h
	../include/box2d/timer.h
	../include/box2d/types.h
)

# Hide internal functions
# todo need to investigate this more
# https://gcc.gnu.org/wiki/Visibility
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

add_library(box2d ${BOX2D_SOURCE_FILES} ${BOX2D_API_FILES})

if (BOX2D_USER_CONSTANTS)
	# this file allows users to override constants
	configure_file(user_constants.h.in user_constants.h)
	target_compile_definitions(box2d PUBLIC BOX2D_USER_CONSTANTS)
endif()

# Generate box2d_export.h to handles shared library builds
# turned this off to make Box2D easier to use without cmake
# include(GenerateExportHeader)
# generate_export_header(box2d)

target_include_directories(box2d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# SIMDE is used to support SIMD math on multiple platforms
target_link_libraries(box2d PRIVATE simde)

# Box2D uses C17
set_target_properties(box2d PROPERTIES
	C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

if (BOX2D_PROFILE)
	target_compile_definitions(box2d PRIVATE BOX2D_PROFILE)

	FetchContent_Declare(
		tracy
		GIT_REPOSITORY https://github.com/wolfpld/tracy.git
		GIT_TAG master
		GIT_SHALLOW TRUE
		GIT_PROGRESS TRUE
	)
	FetchContent_MakeAvailable(tracy)

	target_link_libraries(box2d PUBLIC TracyClient)
endif()

if (MSVC)
	message(STATUS "Box2D on MSVC")	
	if (BUILD_SHARED_LIBS)
		# this is needed by DLL users to import Box2D symbols
		target_compile_definitions(box2d INTERFACE BOX2D_DLL)
	endif()

	# Visual Studio won't load the natvis unless it is in the project
	target_sources(box2d PRIVATE box2d.natvis)

	if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
		# Atomics are still considered experimental in Visual Studio 17.8
		target_compile_options(box2d PRIVATE /experimental:c11atomics)
		target_compile_definitions(box2d PUBLIC "$<$<CONFIG:RELWITHDEBINFO>:B2_ENABLE_ASSERT>")
	elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		message(STATUS "CMake using Clang-CL")
	endif()

	if (BOX2D_AVX2)
		message(STATUS "Box2D using AVX2")	
		target_compile_options(box2d PRIVATE /arch:AVX2)
	endif()
elseif (MINGW)
	message(STATUS "Box2D on MinGW")
	if (BOX2D_AVX2)
		message(STATUS "Box2D using AVX2")	
		target_compile_options(box2d PRIVATE -mavx2)
	else()
		# see SIMDE_DIAGNOSTIC_DISABLE_PSABI_
		message(STATUS "Box2D disabling ABI warning")
		target_compile_options(box2d PRIVATE -Wno-psabi)
	endif()
elseif (APPLE)
	message(STATUS "Box2D on Apple")
	# target_compile_options(box2d PRIVATE -fsanitize=thread -fno-omit-frame-pointer)
elseif (EMSCRIPTEN)
	message(STATUS "Box2D on Emscripten")
	# see SIMDE_DIAGNOSTIC_DISABLE_PSABI_
	message(STATUS "Box2D disabling ABI warning")
	target_compile_options(box2d PRIVATE -Wno-psabi)
elseif (UNIX)
	message(STATUS "Box2D using Unix")
	if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
		# raspberry pi
		# -mfpu=neon
		# target_compile_options(box2d PRIVATE)
	else()
		# x64
		if (BOX2D_AVX2)
			message(STATUS "Box2D using AVX2")
			# FMA -mfma -mavx -march=native
			target_compile_options(box2d PRIVATE -mavx2)
			# target_compile_options(box2d PRIVATE -fsanitize=address)
		else()
			# see SIMDE_DIAGNOSTIC_DISABLE_PSABI_
			message(STATUS "Box2D disabling ABI warning")
			target_compile_options(box2d PRIVATE -Wno-psabi)
		endif()
	endif()
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX2D_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${BOX2D_API_FILES})

install(TARGETS box2d)
