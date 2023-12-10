# Box2D unit test app

set(BOX2D_TESTS
    main.c
    test_bitset.c
    test_collision.c
    test_determinism.c
    test_distance.c
    test_macros.h
    test_math.c
    test_shape.c
    test_table.c
    test_world.c
)

add_executable(test ${BOX2D_TESTS})

# Special access to Box2D internals for testing
target_include_directories(test PRIVATE ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(test PRIVATE box2d enkiTS simde)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "" FILES ${BOX2D_TESTS})
