add_executable(test
    collision_test.c
    distance_test.c
    main.c
    math_test.c
    test_macros.h
)

target_link_libraries(test PUBLIC box2d)
