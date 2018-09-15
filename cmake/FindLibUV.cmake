find_library(LibUV_LIBRARY NAMES uv)
mark_as_advanced(LibUV_LIBRARY)

find_path(LibUV_INCLUDE_DIR NAMES uv.h)

mark_as_advanced(LibUV_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LibUV
        FOUND_VAR LibUV_FOUND
        REQUIRED_VARS LibUV_LIBRARY LibUV_INCLUDE_DIR
        VERSION_VAR LibUV_VERSION
        )
set(LIBUV_FOUND ${LibUV_FOUND})
