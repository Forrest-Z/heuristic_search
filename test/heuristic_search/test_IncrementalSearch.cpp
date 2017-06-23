/* heuristic_search library
 *
 * Copyright (c) 2016, 
 * Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>, 
 * Warsaw University of Technology.
 * All rights reserved.
 *  
 */


#include <gtest/gtest.h>

#include "heuristic_search/SearchLoop.h"
#include "heuristic_search/StdOpenList.h"
#include "heuristic_search/StdSearchSpace.h"
#include "heuristic_search/SearchingAlgorithm.h"
#include "heuristic_search/AStar.h"
#include "heuristic_search/IncrementalSearch.h"

#include "../heuristic_search/test_TestDomain.h"

namespace test_heuristic_search{


typedef heuristic_search::SearchAlgorithmBegin<            
            heuristic_search::SearchLoop<
            heuristic_search::IncrementalSearch_SearchStep<
            heuristic_search::AStar<
            heuristic_search::IncrementalSearch_Initialize<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<TestDomain> > > > > > > > > IncrementalAStarTestAlgorithm_t;
    
TEST(test_IncrementalAStar, search_uninitialized)
{        
    TestDomain domain;
    
    IncrementalAStarTestAlgorithm_t::Algorithm_t algorithm(domain);
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({0},{4}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    ASSERT_TRUE(algorithm.search({0},{5}));    
    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    auto path = algorithm.getStatePath();
    
    ASSERT_EQ(6, path.size());
    
    EXPECT_EQ(TestDomain::State({0}),path[0]);
    EXPECT_EQ(TestDomain::State({1}),path[1]);
    EXPECT_EQ(TestDomain::State({2}),path[2]);
    EXPECT_EQ(TestDomain::State({3}),path[3]);
    EXPECT_EQ(TestDomain::State({4}),path[4]);
    EXPECT_EQ(TestDomain::State({5}),path[5]);

}

    
}
