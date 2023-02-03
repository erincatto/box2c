set(BOX2D_SOURCE_FILES
	allocate.c
	block_allocator.c
	block_allocator.h
	body.c
	body.h
	broad_phase.c
	broad_phase.h
	chain_shape.h
	contact.c
	contact.h
	aabb.c
	distance.c
	dynamic_tree.c
	hull.c
	manifold.c
	math.c
	geometry.c
	polygon_shape.h
	pool.c
	pool.h
	shape.c
	shape.h
	timer.c
	types.c
	world.c
	world.h
)

set(BOX2D_API_FILES
	../include/box2d/allocate.h
	../include/box2d/aabb.h
	../include/box2d/api.h
	../include/box2d/box2d.h
	../include/box2d/callbacks.h
	../include/box2d/constants.h
	../include/box2d/distance.h
	../include/box2d/dynamic_tree.h
	../include/box2d/hull.h
	../include/box2d/id.h
	../include/box2d/manifold.h
	../include/box2d/math.h
	../include/box2d/geometry.h
	../include/box2d/timer.h
	../include/box2d/types.h
)

add_library(box2d ${BOX2D_SOURCE_FILES} ${BOX2D_API_FILES})
target_include_directories(box2d
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(box2d PROPERTIES
	CMAKE_COMPILE_WARNING_AS_ERROR ON
	C_STANDARD 17
	C_STANDARD_REQUIRED YES
	C_EXTENSIONS NO
)

if(MSVC)
	target_compile_options(box2d PRIVATE /W4 /WX)
else()
	target_compile_options(box2d PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
	target_link_libraries(box2d PUBLIC ${MATH_LIBRARY})
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX2D_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${BOX2D_API_FILES})
