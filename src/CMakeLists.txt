# Submodules
include(${CMAKE_SOURCE_DIR}/deps/eigen.cmake)
include(${CMAKE_SOURCE_DIR}/deps/json.cmake)
include(${CMAKE_SOURCE_DIR}/deps/uWebSockets.cmake)
include(${CMAKE_SOURCE_DIR}/deps/CSVWriter.cmake)

add_library(core STATIC
    ${CMAKE_SOURCE_DIR}/src/coordinates.hpp
    ${CMAKE_SOURCE_DIR}/src/coordinates.cpp
    ${CMAKE_SOURCE_DIR}/src/tools.hpp
    ${CMAKE_SOURCE_DIR}/src/tools.cpp
    ${CMAKE_SOURCE_DIR}/src/measurement_package.hpp
    ${CMAKE_SOURCE_DIR}/src/ukf.hpp
    ${CMAKE_SOURCE_DIR}/src/ukf.cpp
    )

target_include_directories(core
    PUBLIC ${CMAKE_SOURCE_DIR}/src
    )

target_link_libraries(core PUBLIC
    Eigen
    json
    libuWS
    CSVWriter
    )

if (DO_CLANG_TIDY)
    set_target_properties(core PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}")
endif ()