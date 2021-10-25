cmake_minimum_required(VERSION 3.1)
project(word_counting)

# Set build type to RELEASE by default:
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type provided, default to RELEASE.")
  set(CMAKE_BUILD_TYPE "RELEASE")
endif()

# Options can be turned on and off by providing cmake with a -DOPTIONNAME=ON (or OFF) argument.
# For example, to turn off benchmarks, build a shared library, and use sanitize flags in a DEBUG build:
#    cmake .. -DCMAKE_BUILD_TYPE=DEBUG -DWORD_COUNTING_BENCH=OFF -DBENCH_STATIC=OFF -DWORD_COUNTING_SANITIZE=ON
option(WORD_COUNTING_TEST      "Build tests" ON)
option(WORD_COUNTING_BENCH     "Build benchmarks" ON)
option(WORD_COUNTING_STATIC    "Builds a static library instead of a shared one" OFF)
option(WORD_COUNTING_SANITIZE  "Adds sanitive flags" OFF)

if(WORD_COUNTING_STATIC)
  set(WORD_COUNTING_LIB_NAME word_counting_static)
  set(WORD_COUNTING_LIB_TYPE STATIC)
else()
  set(WORD_COUNTING_LIB_NAME word_counting)
  set(WORD_COUNTING_LIB_TYPE SHARED)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(Threads REQUIRED)
find_package(Boost)
find_package(Math)

include_directories(. include ${Boost_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 11) # cmake 3.1 supports '98', '11' or '14'. cmake 3.8 or later support '17'.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WORD_COUNTING_SANITIZE)
  set(CMAKE_CXX_FLAGS "-fsanitize=address -fsanitize=undefined ${CMAKE_CXX_FLAGS}")
endif()

add_subdirectory(src)

if(WORD_COUNTING_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

if(WORD_COUNTING_BENCH)
  add_subdirectory(bench)
endif()

message(STATUS "")
message(STATUS "BUILD SUMMARY")
message(STATUS "  CMAKE_GENERATOR      : ${CMAKE_GENERATOR}")
message(STATUS "  Compiler ID          : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "  Build type           : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Library name         : ${WORD_COUNTING_LIB_NAME}")
message(STATUS "  Library type         : ${WORD_COUNTING_LIB_TYPE}")
message(STATUS "  Build tests          : ${WORD_COUNTING_TEST}")
message(STATUS "  Build benchmarks     : ${WORD_COUNTING_BENCH}")
message(STATUS "  Sanitize flags       : ${WORD_COUNTING_SANITIZE}")
message(STATUS "  Boost include dirs   : ${Boost_INCLUDE_DIRS}")
message(STATUS "")
