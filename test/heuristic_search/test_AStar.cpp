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

#include "../heuristic_search/test_TestDomain.h"

namespace test_heuristic_search{


typedef heuristic_search::SearchAlgorithmBegin<            
            heuristic_search::SearchLoop<
            heuristic_search::AStar<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<TestDomain> > > > > > > AStarTestAlgorithm_t;
    
TEST(test_AStar, search_uninitialized)
{        
    TestDomain domain;
    
    AStarTestAlgorithm_t::Algorithm_t algorithm(domain);
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({0},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

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

TEST(test_AStar, search_initialized)
{        
    TestDomain domain;
    
    AStarTestAlgorithm_t::Algorithm_t algorithm(domain);
    
    algorithm.search_space->initialize();
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({0},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

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

TEST(test_AStar, search_initialized_backward)
{        
    TestDomain domain;
    
    AStarTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward);
    
    algorithm.search_space->initialize();
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({0},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

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

   
TEST(test_AStar, search_initialized_updated)
{        
    TestDomain domain;
    
    AStarTestAlgorithm_t::Algorithm_t algorithm(domain);
       
    algorithm.search_space->initialize();
    
    TestDomain::StateActionState updated_action({2},{10*TestDomain::costFactor()},{3});
    domain.updateAction(updated_action);
    
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({0},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    auto path = algorithm.getStatePath();
    
    ASSERT_EQ(6, path.size());
    
    EXPECT_EQ(TestDomain::State({0}),path[0]);
    EXPECT_EQ(TestDomain::State({1}),path[1]);
    EXPECT_EQ(TestDomain::State({6}),path[2]);
    EXPECT_EQ(TestDomain::State({7}),path[3]);
    EXPECT_EQ(TestDomain::State({4}),path[4]);
    EXPECT_EQ(TestDomain::State({5}),path[5]);

}


TEST(test_AStar, search_action_state_path_forward)
{        
    TestDomain domain;
    
    AStarTestAlgorithm_t::Algorithm_t algorithm(domain);
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({0},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

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

TEST(test_AStar, search_action_state_path_backward)
{        
    TestDomain domain;
    
    AStarTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward);
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({0},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

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