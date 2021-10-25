
#include "word_counting/word_counting.hpp"

#include <iostream>
#include <stdexcept>
#include <vector>

namespace word_counting {

auto word_counting(const std::string &corpusPath, const std::string &queryPath)
    -> void {
  auto corpus = Database(corpusPath);
  std::vector<std::string> queryKeys = load_query(queryPath);
  auto queryResults = Database(corpus, queryKeys);
  for (const auto &pr : queryResults.wordCounts()) {
    std::cout << "block: " << pr.first << " " << pr.second << std::endl;
  }
}
} // namespace word_counting
