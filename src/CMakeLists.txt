configure_file(user_constants.h.in user_constants.h)

set(BOX2D_SOURCE_FILES
	aabb.c
	allocate.c
	allocate.h
	arena_allocator.c
	arena_allocator.h
	array.c
	array.h
	bitset.c
	bitset.h
	bitset.inl
	block_allocator.c
	block_allocator.h
	body.c
	body.h
	broad_phase.c
	broad_phase.h
	contact.c
	contact.h
	contact_solver.c
	contact_solver.h
	core.c
	core.h
	distance.c
	distance_joint.c
	dynamic_tree.c
	dynamic_tree_types.h
	geometry.c
	graph.c
	graph.h
	hull.c
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
	solver_data.h
	table.c
	table.h
	timer.c
	types.c
	user_constants.h.in
	weld_joint.c
	wheel_joint.c
	world.c
	world.h
)

set(BOX2D_API_FILES
	../include/box2d/aabb.h
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
	../include/box2d/joint_util.h
	../include/box2d/manifold.h
	../include/box2d/math.h
	../include/box2d/timer.h
	../include/box2d/types.h
)

if (MSVC)
	add_library(box2d ${BOX2D_SOURCE_FILES} ${BOX2D_API_FILES} box2d.natvis)
else()
	add_library(box2d ${BOX2D_SOURCE_FILES} ${BOX2D_API_FILES})
endif()

target_include_directories(box2d
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include ${CMAKE_CURRENT_BINARY_DIR}
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(box2d PRIVATE simde)

# box2d uses C17
set_target_properties(box2d PROPERTIES
	C_STANDARD 17
	C_STANDARD_REQUIRED YES
	C_EXTENSIONS YES
)

if (MSVC)
	if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
		# target_compile_options(box2d PRIVATE /W4 /WX)
		target_compile_options(box2d PRIVATE /W4 /WX /experimental:c11atomics)
		target_compile_options(box2d PRIVATE /arch:AVX2)
		# target_compile_options(box2d PRIVATE /arch:SSE2)
		target_compile_definitions(box2d PUBLIC "$<$<CONFIG:RELWITHDEBINFO>:B2_ENABLE_ASSERT>")
	elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		message(STATUS "CMake using Clang-CL")
		target_compile_options(box2d PRIVATE /W4 /WX /std:c17 /arch:AVX2)
	endif()
elseif (LINUX)
	if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
		# raspberry pi
		target_compile_options(box2d PRIVATE -Wall -Wextra -Wpedantic -Werror)
	else()
		# FMA -mfma
		target_compile_options(box2d PRIVATE -Wall -Wextra -Wpedantic -Werror -mavx)
	endif()
elseif (APPLE)
	target_compile_options(box2d PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

target_compile_definitions(box2d PRIVATE B2_UNIT_OVERRIDE=${BOX2D_UNIT_OVERRIDE})

# todo needed?
find_library(MATH_LIBRARY m)
if (MATH_LIBRARY)
	target_link_libraries(box2d PUBLIC ${MATH_LIBRARY})
endif()

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

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX2D_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${BOX2D_API_FILES})
