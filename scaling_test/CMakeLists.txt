# Box2D scaling_test app

add_executable(scaling_test 
    main.c
    timer.h
    timer.c
)

set_target_properties(scaling_test PROPERTIES
	C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
)

# Special access to Box2D internals for testing
# target_include_directories(scaling_test PRIVATE ${CMAKE_SOURCE_DIR}/src)

if (MSVC)
    target_compile_options(scaling_test PRIVATE /experimental:c11atomics)
endif()

target_link_libraries(scaling_test PRIVATE enkiTS)
