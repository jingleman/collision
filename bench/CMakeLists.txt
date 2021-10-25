set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Suppressing benchmark's tests" FORCE)

# add_subdirectory(benchmark)
# include_directories(benchmark/include)
find_package(benchmark REQUIRED)
#include_directories(${BENCHMARK_INCLUDE_DIRS})

set(bench_cpp
  word_counting_bench.cpp
)

add_executable(bench_word_counting ${bench_cpp})

target_link_libraries(bench_word_counting
        #benchmark
  benchmark::benchmark
  ${WORD_COUNTING_LIB_NAME}
  ${CMAKE_THREAD_LIBS_INIT}
  ${MATH_LIBS}
)
