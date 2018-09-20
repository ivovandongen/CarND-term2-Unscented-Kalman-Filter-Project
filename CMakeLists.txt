cmake_minimum_required(VERSION 3.5)
project(UnscentedKF)

set(CMAKE_CXX_STANDARD 14)

# CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Ensure git submodules are up to date
include(${CMAKE_MODULE_PATH}/git.cmake)

if (UNIX)
    # Silence warnings in Eigen
    add_compile_options(-Wno-deprecated-register)
endif ()

# Clang format

file(GLOB_RECURSE SRC_FILES src/*.cpp src/*.hpp)
file(GLOB_RECURSE TEST_FILES src/*.cpp src/*.hpp)
set(ALL_SOURCES ${SRC_FILES} ${TEST_FILES})

find_program(CLANG_FORMAT NAMES "clang-format" DOC "Path to clang-format executable")
if (NOT CLANG_FORMAT)
    message(STATUS "clang-format not found.")
else ()
    message(STATUS "clang-format found: ${CLANG_FORMAT}")
    add_custom_target(
        clang-format
        COMMAND ${CLANG_FORMAT}
        -style=file
        -i
        ${ALL_SOURCES}
    )
endif ()

# Clang tidy

find_program(CLANG_TIDY NAMES "clang-tidy" DOC "Path to clang-tidy executable")
if (NOT CLANG_TIDY)
    message(STATUS "clang-tidy not found.")
else ()
    message(STATUS "clang-tidy found: ${CLANG_TIDY}")
    set(DO_CLANG_TIDY "${CLANG_TIDY}" "-fix;-fix-errors")
endif ()

# Core sources

add_subdirectory(src)

# Main executable

add_executable(UnscentedKF
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    )

target_link_libraries(UnscentedKF core)

#if (DO_CLANG_TIDY)
#    set_target_properties(UnscentedKF PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}")
#endif ()

# Tests

add_subdirectory(test)

