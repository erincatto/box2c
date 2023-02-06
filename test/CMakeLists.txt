set(BOX2D_TESTS
    main.c
    test_collision.c
    test_distance.c
    test_macros.h
    test_math.c
    test_shape.c
    test_world.c
)

add_executable(test ${BOX2D_TESTS})
target_link_libraries(test PUBLIC box2d)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "" FILES ${BOX2D_TESTS})
