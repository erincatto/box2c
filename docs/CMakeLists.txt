find_package(Doxygen REQUIRED dot)

set(DOXYGEN_EXTRACT_ALL NO)
set(DOXYGEN_FILE_PATTERNS *.h)
set(DOXYGEN_ENABLE_PREPROCESSING YES)
set(DOXYGEN_MACRO_EXPANSION YES)
set(DOXYGEN_EXPAND_ONLY_PREDEF YES)
set(DOXYGEN_PREDEFINED BOX2D_API=)

set(DOXYGEN_IMAGE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/images")
set(DOXYGEN_HTML_EXTRA_STYLESHEET "${CMAKE_CURRENT_SOURCE_DIR}/extra.css")
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "${CMAKE_CURRENT_SOURCE_DIR}/overview.md")
set(DOXYGEN_PROJECT_LOGO "${CMAKE_CURRENT_SOURCE_DIR}/images/logo.svg")

set(DOXYGEN_INLINE_SIMPLE_STRUCTS YES)
set(DOXYGEN_TYPEDEF_HIDES_STRUCT YES)
set(DOXYGEN_DISABLE_INDEX YES)
set(DOXYGEN_GENERATE_TREEVIEW YES)
set(DOXYGEN_FULL_SIDEBAR NO)

# this tells doxygen to label structs as structs intead of classes
set(DOXYGEN_OPTIMIZE_OUTPUT_FOR_C YES)

set(DOXYGEN_WARN_IF_INCOMPLETE_DOC NO)

doxygen_add_docs(doc
                "${CMAKE_SOURCE_DIR}/include/box2d"
                "overview.md"
                "hello.md"
                "testbed.md"
                "common.md"
                "collision.md"
                "dynamics.md"
                "loose_ends.md"
                "references.md"
                "FAQ.md"
                ALL
                COMMENT "Generate HTML documentation")
