add_subdirectory(gtest)

include_directories(./gtest/googletest/include)

set(test_src
  run_all.cpp
  word_counting_test.cpp
)

add_executable(test_word_counting ${test_src})

target_link_libraries(test_word_counting
  gtest
  ${WORD_COUNTING_LIB_NAME}
  ${CMAKE_THREAD_LIBS_INIT}
  ${MATH_LIBS}
)

add_test(
  NAME tests
  COMMAND test_word_counting
)
