cmake_minimum_required(VERSION 3.12)
project(pidf CXX)

include(FetchContent)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master) # or whatever tag you want
FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()


# Add an executable for testing
add_executable(test_pidf test.cpp pidf.h)
target_link_libraries(test_pidf PUBLIC matplot)
