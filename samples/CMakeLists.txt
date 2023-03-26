find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_path(JSMN_INCLUDE_DIRS "jsmn.h")

set(BOX2D_SAMPLES
	draw.cpp
	draw.h
	main.cpp
	sample.cpp
	sample.h
	settings.h
	settings.cpp

	collection/benchmark_barrel.cpp
	collection/benchmark_joint_grid.cpp
	collection/benchmark_pyramid.cpp
	collection/benchmark_tumbler.cpp

	collection/sample_distance.cpp
	collection/sample_dynamic_tree.cpp
	collection/sample_hull.cpp
	collection/sample_manifold.cpp
	collection/sample_ray_cast.cpp
	collection/sample_revolute_joint.cpp
	collection/sample_shape_cast.cpp
	collection/sample_time_of_impact.cpp
	collection/sample_vertical_stack.cpp
)

add_executable(samples ${BOX2D_SAMPLES})

set_target_properties(samples PROPERTIES
	CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

target_include_directories(samples PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${JSMN_INCLUDE_DIRS})
target_link_libraries(samples PUBLIC box2d glfw imgui::imgui glad::glad)

# message(STATUS "runtime = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
# message(STATUS "binary = ${CMAKE_CURRENT_BINARY_DIR}")

# Copy font files, etc
add_custom_command(
	TARGET samples POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory
			${CMAKE_CURRENT_SOURCE_DIR}/data/
			${CMAKE_CURRENT_BINARY_DIR}/data/)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${BOX2D_SAMPLES})
