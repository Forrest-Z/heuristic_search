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
#include "heuristic_search/DStarExtraLite.h"
#include "heuristic_search/DStarMain.h"

#include "../heuristic_search/test_TestDomain.h"

namespace test_heuristic_search{


typedef heuristic_search::SearchAlgorithmBegin<            
            heuristic_search::DStarMain<
            heuristic_search::SearchLoop<
            heuristic_search::DStarExtraLite<
            heuristic_search::IncrementalSearch_SearchStep<
            heuristic_search::AStar<
            heuristic_search::IncrementalSearch_Initialize<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<TestDomain> > > > > > > > > > > DStarExtraLiteTestAlgorithm_t;
    
TEST(test_DStarExtraLite, search_reinitialize_start)
{        
    TestDomain domain;
    
    DStarExtraLiteTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward);
       
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
    
TEST(test_DStarExtraLite, search_reinitialize_start_in_range)
{        
    TestDomain domain;
    
    DStarExtraLiteTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward);
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({0},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    std::vector<std::pair<TestDomain::StateActionState, TestDomain::Cost> > updated_actions;
    algorithm.reinitialize({1}, updated_actions);
    ASSERT_TRUE(algorithm.search());    
    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    auto path = algorithm.getStatePath();
    
    ASSERT_EQ(5, path.size());
    
    EXPECT_EQ(TestDomain::State({1}),path[0]);
    EXPECT_EQ(TestDomain::State({2}),path[1]);
    EXPECT_EQ(TestDomain::State({3}),path[2]);
    EXPECT_EQ(TestDomain::State({4}),path[3]);
    EXPECT_EQ(TestDomain::State({5}),path[4]);

}
    
TEST(test_DStarExtraLite, search_cost_increase_backward)
{       
    typedef DStarExtraLiteTestAlgorithm_t::Algorithm_t::Node_t Node_t;
    
    TestDomain domain;
    
    DStarExtraLiteTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward, true);
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({0},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    std::vector<std::pair<TestDomain::StateActionState, TestDomain::Cost> > updated_actions;
    updated_actions.push_back(
        std::make_pair(TestDomain::StateActionState(
                                {2},{10*TestDomain::costFactor()},{3}), TestDomain::costFactor()));
    domain.updateAction(TestDomain::StateActionState(
                                {2},{10*TestDomain::costFactor()},{3}));
    
    algorithm.reinitialize({1}, updated_actions);
    
    EXPECT_FALSE(algorithm.goal_node->visited);
    EXPECT_FALSE(algorithm.goal_node->open);
    EXPECT_FALSE(algorithm.goal_node->closed);
    
    Node_t *node_0 = algorithm.search_space->getNode({0});    
    EXPECT_FALSE(node_0->visited);
    EXPECT_FALSE(node_0->open);
    EXPECT_FALSE(node_0->closed);
    
    Node_t *node_2 = algorithm.search_space->getNode({2});    
    EXPECT_FALSE(node_2->visited);
    EXPECT_FALSE(node_2->open);
    EXPECT_FALSE(node_2->closed);
    
    Node_t *node_3 = algorithm.search_space->getNode({3});    
    EXPECT_TRUE(node_3->visited);
    EXPECT_TRUE(node_3->open);
    EXPECT_FALSE(node_3->closed);
    
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
 
 
TEST(test_DStarExtraLite, search_cost_increase_no_need_for_replanning)
{       
    
    TestDomain domain;
    
    DStarExtraLiteTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward, true);
       
    ASSERT_FALSE(algorithm.initialized);
    ASSERT_FALSE(algorithm.finished);
    ASSERT_FALSE(algorithm.found);

    ASSERT_TRUE(algorithm.search({0},{5}));
    ASSERT_TRUE(algorithm.start_node->visited);
    ASSERT_TRUE(algorithm.goal_node->visited);

    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    std::vector<std::pair<TestDomain::StateActionState, TestDomain::Cost> > updated_actions;
    updated_actions.push_back(
        std::make_pair(TestDomain::StateActionState(
                                {7},{10*TestDomain::costFactor()},{4}), TestDomain::costFactor()));
    
    algorithm.reinitialize({1}, updated_actions);
    
    EXPECT_TRUE(algorithm.goal_node->visited);
    EXPECT_FALSE(algorithm.goal_node->open);
    EXPECT_TRUE(algorithm.goal_node->closed);
            
    
    EXPECT_TRUE(algorithm.solutionFound());
    
    EXPECT_TRUE(algorithm.search());    
    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    auto path = algorithm.getStatePath();
    
    ASSERT_EQ(5, path.size());
    
    EXPECT_EQ(TestDomain::State({1}),path[0]);
    EXPECT_EQ(TestDomain::State({2}),path[1]);
    EXPECT_EQ(TestDomain::State({3}),path[2]);
    EXPECT_EQ(TestDomain::State({4}),path[3]);
    EXPECT_EQ(TestDomain::State({5}),path[4]);

}


TEST(test_DStarExtraLite, search_cost_decrease_backward)
{       
    typedef DStarExtraLiteTestAlgorithm_t::Algorithm_t::Node_t Node_t;
    
    TestDomain domain;
    
    DStarExtraLiteTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward, true);
        
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
    
    auto old_path = algorithm.getStatePath();
    
    ASSERT_EQ(6, old_path.size());
    
    EXPECT_EQ(TestDomain::State({0}),old_path[0]);
    EXPECT_EQ(TestDomain::State({1}),old_path[1]);
    EXPECT_EQ(TestDomain::State({6}),old_path[2]);
    EXPECT_EQ(TestDomain::State({7}),old_path[3]);
    EXPECT_EQ(TestDomain::State({4}),old_path[4]);
    EXPECT_EQ(TestDomain::State({5}),old_path[5]);
                           
    TestDomain::StateActionState updated_action_2({2},{1*TestDomain::costFactor()},{3});
    domain.updateAction(updated_action_2);
    
    std::vector<std::pair<TestDomain::StateActionState, TestDomain::Cost> > updated_actions;
    updated_actions.push_back(std::make_pair(updated_action_2, 10*TestDomain::costFactor()));
             
    algorithm.reinitialize({1}, updated_actions);
    
    EXPECT_TRUE(algorithm.goal_node->visited);
    EXPECT_TRUE(algorithm.goal_node->open);
    EXPECT_FALSE(algorithm.goal_node->closed);
        
    Node_t *node_3 = algorithm.search_space->getNode({3});    
    EXPECT_TRUE(node_3->visited);
    EXPECT_TRUE(node_3->open);
    EXPECT_FALSE(node_3->closed);
    
    ASSERT_TRUE(algorithm.search());    
    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    auto path = algorithm.getStatePath();
    
    ASSERT_EQ(5, path.size());
    
    EXPECT_EQ(TestDomain::State({1}),path[0]);
    EXPECT_EQ(TestDomain::State({2}),path[1]);
    EXPECT_EQ(TestDomain::State({3}),path[2]);
    EXPECT_EQ(TestDomain::State({4}),path[3]);
    EXPECT_EQ(TestDomain::State({5}),path[4]);

}

TEST(test_DStarExtraLite, search_cost_decrease_backward_no_need_for_searching)
{       
    typedef DStarExtraLiteTestAlgorithm_t::Algorithm_t::Node_t Node_t;
    
    TestDomain domain;
    
    DStarExtraLiteTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward, true);
        
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
    
    auto old_path = algorithm.getStatePath();
    
    ASSERT_EQ(6, old_path.size());
    
    EXPECT_EQ(TestDomain::State({0}),old_path[0]);
    EXPECT_EQ(TestDomain::State({1}),old_path[1]);
    EXPECT_EQ(TestDomain::State({6}),old_path[2]);
    EXPECT_EQ(TestDomain::State({7}),old_path[3]);
    EXPECT_EQ(TestDomain::State({4}),old_path[4]);
    EXPECT_EQ(TestDomain::State({5}),old_path[5]);
    
    TestDomain::StateActionState updated_action_2({2},{1*TestDomain::costFactor()},{3});
    domain.updateAction(updated_action_2);
    std::vector<std::pair<TestDomain::StateActionState, TestDomain::Cost> > updated_actions;
    updated_actions.push_back(std::make_pair(updated_action_2, 10*TestDomain::costFactor()));
    
    algorithm.reinitialize({6}, updated_actions);
    
    EXPECT_TRUE(algorithm.goal_node->visited);
    EXPECT_FALSE(algorithm.goal_node->open);
    EXPECT_TRUE(algorithm.goal_node->closed);
        
    Node_t *node_3 = algorithm.search_space->getNode({3});    
    EXPECT_TRUE(node_3->visited);
    EXPECT_TRUE(node_3->open);
    EXPECT_FALSE(node_3->closed);
    
    ASSERT_TRUE(algorithm.search());    
    EXPECT_TRUE(algorithm.finished);
    EXPECT_TRUE(algorithm.found);
    
    auto path = algorithm.getStatePath();
    
    ASSERT_EQ(4, path.size());
    
    EXPECT_EQ(TestDomain::State({6}),path[0]);
    EXPECT_EQ(TestDomain::State({7}),path[1]);
    EXPECT_EQ(TestDomain::State({4}),path[2]);
    EXPECT_EQ(TestDomain::State({5}),path[3]);

}

TEST(test_DStarExtraLite, main_backward)
{        
    TestDomain domain;
    
    DStarExtraLiteTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward);
           
    algorithm.main({0},{5});
    
    EXPECT_EQ(TestDomain::State(5), algorithm.current_state);

}


}
