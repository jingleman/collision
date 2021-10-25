
#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include "word_counting/word_counting.hpp"

auto main(int argc, char **argv) -> int {
  try {
    if (argc != 3) {
      std::stringstream ss;
      ss << "usage: " << argv[0] << " " << "<corpus_path>"<< " " << "<query_path>";
      throw std::invalid_argument(ss.str());
    }   // Also check values?
    word_counting::word_counting(std::string(argv[1]), std::string(argv[2]));
  } catch (const std::exception &e) {
    std::cerr << "Caught exception: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}

