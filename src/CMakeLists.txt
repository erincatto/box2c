set(BOX2D_SOURCE_FILES
	allocate.c
	block_allocator.c
	block_allocator.h
	collision.c
	distance.c
	timer.c
	vec_math.c
)

set(BOX2D_HEADER_FILES
	../include/box2d/allocate.h
	../include/box2d/chain_shape.h
	../include/box2d/circle_shape.h
	../include/box2d/collision.h
	../include/box2d/constants.h
	../include/box2d/distance.h
	../include/box2d/edge_shape.h
	../include/box2d/polygon_shape.h
	../include/box2d/shape.h
	../include/box2d/timer.h
	../include/box2d/types.h
	../include/box2d/vec_math.h
)

add_library(box2d ${BOX2D_SOURCE_FILES} ${BOX2D_HEADER_FILES})
target_include_directories(box2d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
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

# source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX2D_SOURCE_FILES})
# source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${BOX2D_HEADER_FILES})
