set(BOX2D_SOURCE_FILES
	aabb.c
	allocate.c
	allocate.h
	array.c
	array.h
	bitset.c
	bitset.h
	block_allocator.c
	block_allocator.h
	body.c
	body.h
	broad_phase.c
	broad_phase.h
	chain_shape.h
	contact.c
	contact.h
	contact_solver.c
	contact_solver.h
	core.c
	core.h
	distance.c
	dynamic_tree.c
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
	mouse_joint.c
	polygon_shape.h
	pool.c
	pool.h
	revolute_joint.c
	shape.c
	shape.h
	solver_data.h
	stack_allocator.c
	stack_allocator.h
	table.c
	table.h
	timer.c
	types.c
	weld_joint.c
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

add_library(box2d ${BOX2D_SOURCE_FILES} ${BOX2D_API_FILES})
target_include_directories(box2d
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

message(STATUS "CMake C compiler: ${CMAKE_C_COMPILER_ID}")
message(STATUS "CMake C++ compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMake system name: ${CMAKE_SYSTEM_NAME}")

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
	if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
		# target_compile_options(box2d PRIVATE /W4 /WX)
		target_compile_options(box2d PRIVATE /W4 /WX /experimental:c11atomics)
		target_compile_options(box2d PRIVATE /arch:AVX)
		# target_compile_options(box2d PRIVATE /arch:SSE2 /fp:except-)
		# target_compile_definitions(box2d PUBLIC "$<$<CONFIG:RELWITHDEBINFO>:B2_ENABLE_ASSERT>")
	elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		message(STATUS "CMake using Clang-CL")
		target_compile_options(box2d PRIVATE /W4 /WX /std:c17 /arch:AVX)
	endif()
else()
	target_compile_options(box2d PRIVATE -Wall -Wextra -Wpedantic -Werror -mavx)
endif()

find_library(MATH_LIBRARY m)
if (MATH_LIBRARY)
	target_link_libraries(box2d PUBLIC ${MATH_LIBRARY})
endif()

if (BOX2D_PROFILE)
	target_link_libraries(box2d PUBLIC TracyClient)
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX2D_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${BOX2D_API_FILES})
