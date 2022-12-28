add_executable(test
    collision_test.c
    main.c
    math_test.c
)

find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
    target_link_libraries(test PUBLIC ${MATH_LIBRARY})
endif()

target_link_libraries(test PUBLIC box2d)
