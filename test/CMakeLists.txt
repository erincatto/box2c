set(BOX2D_TESTS
    collision_test.c
    distance_test.c
    hello_world.c
    main.c
    math_test.c
    shape_test.c
    test_macros.h
)

add_executable(test ${BOX2D_TESTS})
target_link_libraries(test PUBLIC box2d)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "" FILES ${BOX2D_TESTS})
