if (GIT_EXECUTABLE AND EXISTS "${GIT_EXECUTABLE}")
    set(GIT_FOUND TRUE)
else ()
    find_package(Git QUIET)

    # Try some alternate paths for macos installs
    if (NOT GIT_FOUND)
        message("git not found. Trying alternatives")
        if (EXISTS "/usr/local/bin/git")
            set(GIT_EXECUTABLE "/usr/local/bin/git")
            set(GIT_FOUND TRUE)
        elseif(EXISTS "/usr/bin/git")
            set(GIT_EXECUTABLE "/usr/bin/git")
            set(GIT_FOUND TRUE)
        else()
            message("No alternative path could be found for git")
        endif()
    endif()
endif()

if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    message(STATUS "Using git from ${GIT_EXECUTABLE}")

    # Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    else()
        message("No git submodules found")
    endif()
else()
    message(FATAL_ERROR "Could not find git. Try setting the path to the executable with -DGIT_EXECUTABLE=/path/to/git")
endif()