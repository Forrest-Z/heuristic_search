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
#include "heuristic_search/RepeatedAStar.h"
#include "heuristic_search/DStarMain.h"

#include "../heuristic_search/test_TestDomain.h"

namespace test_heuristic_search{


typedef heuristic_search::SearchAlgorithmBegin<            
            heuristic_search::DStarMain<
            heuristic_search::SearchLoop<
            heuristic_search::RepeatedAStar<
            heuristic_search::AStar<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<TestDomain> > > > > > > > > RepeatedAStarTestAlgorithm_t;
    
TEST(test_RepeatedAStar, search_reinitialize_start)
{        
    TestDomain domain;
    
    RepeatedAStarTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward, true);
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({1},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    std::vector<std::pair<TestDomain::StateActionState, TestDomain::Cost> > updated_actions;
    algorithm.reinitialize({0}, updated_actions);
    ASSERT_TRUE(algorithm.search());    
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
        
TEST(test_RepeatedAStar, search_cost_increase_backward)
{           
    TestDomain domain;
    
    RepeatedAStarTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward, true);
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({0},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    TestDomain::StateActionState updated_action({2},{10*TestDomain::costFactor()},{3});
    domain.updateAction(updated_action);
    
    std::vector<std::pair<TestDomain::StateActionState, TestDomain::Cost> > updated_actions;
    updated_actions.push_back(std::make_pair(updated_action, 1*TestDomain::costFactor()));
    
    algorithm.reinitialize({1}, updated_actions);
            
    ASSERT_TRUE(algorithm.search());    
    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    auto path = algorithm.getStatePath();
    
    ASSERT_EQ(5, path.size());
    
    EXPECT_EQ(TestDomain::State({1}),path[0]);
    EXPECT_EQ(TestDomain::State({6}),path[1]);
    EXPECT_EQ(TestDomain::State({7}),path[2]);
    EXPECT_EQ(TestDomain::State({4}),path[3]);
    EXPECT_EQ(TestDomain::State({5}),path[4]);

}
    
TEST(test_RepeatedAStar, search_cost_increase_forward)
{           
    TestDomain domain;
    
    RepeatedAStarTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Forward, true);
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({1},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    TestDomain::StateActionState updated_action({2},{10*TestDomain::costFactor()},{3});
    domain.updateAction(updated_action);
    
    std::vector<std::pair<TestDomain::StateActionState, TestDomain::Cost> > updated_actions;
    updated_actions.push_back(std::make_pair(updated_action, 1*TestDomain::costFactor()));
    
    algorithm.reinitialize({1}, updated_actions);
        
    ASSERT_TRUE(algorithm.search());    
    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    auto path = algorithm.getStatePath();
    
    ASSERT_EQ(5, path.size());
    
    EXPECT_EQ(TestDomain::State({1}),path[0]);
    EXPECT_EQ(TestDomain::State({6}),path[1]);
    EXPECT_EQ(TestDomain::State({7}),path[2]);
    EXPECT_EQ(TestDomain::State({4}),path[3]);
    EXPECT_EQ(TestDomain::State({5}),path[4]);

}


TEST(test_RepeatedAStar, main_backward)
{        
    TestDomain domain;
    
    RepeatedAStarTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward);
           
    algorithm.main({0},{5});
    
    EXPECT_EQ(TestDomain::State(5), algorithm.current_state);

}

TEST(test_RepeatedAStar, main_forward)
{        
    TestDomain domain;
    
    RepeatedAStarTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Forward);
           
    algorithm.main({0},{5});
    
    EXPECT_EQ(TestDomain::State(5), algorithm.current_state);

}

}

