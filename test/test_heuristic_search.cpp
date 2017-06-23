/* heuristic_search library
 *
 * Copyright (c) 2016,
 * Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>,
 * Warsaw University of Technology.
 * All rights reserved.
 *
 */

#include <iostream>

#include "gtest/gtest.h"

#include "../test/heuristic_search/test_DStarMain.cpp"
#include "../test/heuristic_search/test_StdSearchSpace.cpp"
#include "../test/heuristic_search/test_HeuristicSearch.cpp"
#include "../test/heuristic_search/test_StdOpenList.cpp"
#include "../test/heuristic_search/test_AStar.cpp"
#include "../test/heuristic_search/test_IncrementalSearch.cpp"
#include "../test/heuristic_search/test_DStarExtraLite.cpp"
#include "../test/heuristic_search/test_DStarLiteOptimized.cpp"
#include "../test/heuristic_search/test_DStarLite.cpp"
#include "../test/heuristic_search/test_MPGAAStar.cpp"
#include "../test/heuristic_search/test_RepeatedAStar.cpp"

#include "../test/heuristic_search/loggers/test_Logger.cpp"

#include "../test/heuristic_search/benchmark/test_Benchmark.cpp"

#include "../test/search_domains/movingai_benchmark/test_Domain.cpp"
#include "../test/search_domains/movingai_benchmark/test_DStarLiteOptimized.cpp"
#include "../test/search_domains/movingai_benchmark/test_DStarLite.cpp"
#include "../test/search_domains/movingai_benchmark/test_MPGAAStar.cpp"

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
