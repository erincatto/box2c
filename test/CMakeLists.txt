add_executable(test
    main.c
    math_test.c
)

target_link_libraries(test PUBLIC box2d)

#source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES
#    math_test.c
#)
