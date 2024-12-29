cmake_minimum_required(VERSION 3.25)

project(robotics)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(OPENSSL_USE_STATIC_LIBS TRUE)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

SET(GCC_COVERAGE_COMPILE_FLAGS "-Wall -Wextra -Wno-missing-field-initializers -Wno-parentheses -Wno-missing-braces\
                                -Wno-unused-local-typedefs -Wfloat-equal -Wpointer-arith -Wcast-align\
                                -Wnull-dereference -Wimplicit-fallthrough -Wnon-virtual-dtor -Wmissing-declarations\
                                -Werror=multichar -Werror=sizeof-pointer-memaccess -Werror=return-type\
                                -Werror=delete-non-virtual-dtor -fstrict-aliasing -Wsuggest-final-types\
                                -Wsuggest-final-methods -Wsuggest-override -Wpessimizing-move -Wredundant-move")
SET(GCC_COVERAGE_LINK_FLAGS "")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

# add f-analyzer
if( CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
    # set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fanalyzer -coverage")
    # Using this mode results in a compilation error due to too much RAM consumption (over 60 GB)
elseif( CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_DEBUG "")
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fanalyzer -coverage")
    # Using this mode results in a compilation error due to too much RAM consumption (over 60 GB)
else()
    MESSAGE( STATUS "CMAKE_BUILD_TYPE not set yet ${CMAKE_BUILD_TYPE}" )
endif()

# Debug & Release Builds
if(${CMAKE_BUILD_TYPE} MATCHES "(R|r)elease")
    message(STATUS "CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
elseif(${CMAKE_BUILD_TYPE} MATCHES "(D|d)ebug")
    message(STATUS "CMAKE_CXX_FLAGS_DEBUG=${CMAKE_CXX_FLAGS_DEBUG}")
endif()

# Clang-tidy
#issues with ranges, need to remove ranges from checks
#find_program(CLANG_TIDY_EXE NAMES clang-tidy)
#set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}" "-extra-arg=-std=c++2b" "-checks=-*,performance-*")


# Address Sanitizer
if(${CMAKE_BUILD_TYPE} MATCHES "(D|d)ebug")
    set(CMAKE_CXX_FLAGS_ASAN
        "-fsanitize=address -fno-optimize-sibling-calls -fsanitize-address-use-after-scope -fno-omit-frame-pointer -g -O1"
        CACHE STRING "Flags used by the C++ compiler during AddressSanitizer builds." FORCE)
endif()

add_subdirectory(src)

#include(CTest)
#if (BUILD_TESTING) 
#    enable_testing()
#    add_subdirectory(tests)
#endif()


