cmake_minimum_required(VERSION 3.15)
project(PointInPolyhedron VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(point_query
    src/main.cpp
)

target_include_directories(point_query PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(test_stl_parser
    tests/test_stl_parser.cpp
)
add_executable(test_classifier
    tests/test_classifier.cpp
)

target_include_directories(test_stl_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_include_directories(test_classifier PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(MSVC)
    target_compile_options(point_query PRIVATE
        /W4
        $<$<CONFIG:Debug>:/Od /Zi>
        $<$<CONFIG:Release>:/O2>
    )
else()
    target_compile_options(point_query PRIVATE
        -Wall -Wextra -Wpedantic
        -Wshadow -Wconversion -Wsign-conversion
        $<$<CONFIG:Debug>:-O0 -g3 -fno-omit-frame-pointer>
        $<$<CONFIG:Release>:-O3>
    )
endif()

enable_testing()
add_test(NAME stl_parser_unit COMMAND test_stl_parser)
add_test(NAME classifier_unit COMMAND test_classifier)
