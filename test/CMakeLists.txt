add_executable(test
    collision_test.c
    main.c
    math_test.c
)

target_link_libraries(test PUBLIC box2d)
