/* heuristic_search library
 *
 * Copyright (c) 2016,
 * Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>,
 * Warsaw University of Technology.
 * All rights reserved.
 *
 */

#include <gtest/gtest.h>

#include "heuristic_search/StdSearchSpace.h"
#include "heuristic_search/SearchingAlgorithm.h"

#include "../heuristic_search/test_TestDomain.h"

namespace test_heuristic_search{

typedef heuristic_search::SearchAlgorithmBegin<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<TestDomain> > > SearchingAlgorithm_t;

TEST(test_StdSearchSpace, get_node_uninitialized)
{
    TestDomain domain;

    SearchingAlgorithm_t::SearchSpace_t search_space(domain);

    SearchingAlgorithm_t::Node_t *node = search_space.getNode({0});
    EXPECT_NE(nullptr, node);


}

TEST(test_StdSearchSpace, get_node_initialized_out_of_bound)
{
    TestDomain domain;

    SearchingAlgorithm_t::SearchSpace_t search_space(domain);

    search_space.initialize();

    SearchingAlgorithm_t::Node_t *node = search_space.getNode({-1});

    EXPECT_EQ(nullptr, node);


}

TEST(test_StdSearchSpace, get_node_initialized)
{
    TestDomain domain;

    SearchingAlgorithm_t::SearchSpace_t search_space(domain);

    search_space.initialize();

    SearchingAlgorithm_t::Node_t *node = search_space.getNode({0});

    ASSERT_NE(nullptr, node);
    EXPECT_TRUE(node->expanded_succs);
    EXPECT_TRUE(node->expanded_preds);
    ASSERT_FALSE((node->predecessors.empty()));

    int pred_count = node->predecessors.size();
    EXPECT_EQ(1 ,pred_count);

    int succ_count = node->successors.size();
    EXPECT_EQ(1 ,succ_count);

    ASSERT_FALSE((node->successors.empty()));
    EXPECT_EQ(TestDomain::State(0), node->state);
    EXPECT_EQ(TestDomain::State(1), node->successors.front()->state);
    EXPECT_EQ(TestDomain::State(1), node->predecessors.front()->state);

}

TEST(test_StdSearchSpace, get_predecessors_unitialized)
{
    TestDomain domain;

    SearchingAlgorithm_t::SearchSpace_t search_space(domain);

    SearchingAlgorithm_t::Node_t *node = search_space.getNode({0});

    std::vector<SearchingAlgorithm_t::Node_t*> predecessors = search_space.predecessors(node);

    ASSERT_NE(nullptr, node);
    EXPECT_TRUE(node->expanded_preds);
    ASSERT_FALSE((predecessors.empty()));
    EXPECT_EQ(TestDomain::State(0), node->state);
    EXPECT_EQ(TestDomain::State(1), predecessors.front()->state);

}

TEST(test_StdSearchSpace, get_successors_unitialized)
{
    TestDomain domain;

    SearchingAlgorithm_t::SearchSpace_t search_space(domain);

    SearchingAlgorithm_t::Node_t *node = search_space.getNode({0});

    std::vector<SearchingAlgorithm_t::Node_t*> successors = search_space.successors(node);

    ASSERT_NE(nullptr, node);
    EXPECT_TRUE(node->expanded_succs);
    ASSERT_FALSE((successors.empty()));
    EXPECT_EQ(TestDomain::State(0), node->state);
    EXPECT_EQ(TestDomain::State(1), successors.front()->state);

}


TEST(test_StdSearchSpace, update_action)
{
    TestDomain domain;

    SearchingAlgorithm_t::SearchSpace_t search_space(domain);

    search_space.initialize();

    SearchingAlgorithm_t::Node_t *node = search_space.getNode({0});

    ASSERT_NE(nullptr, node);
    EXPECT_TRUE(node->expanded_preds);
    EXPECT_TRUE(node->expanded_succs);
    ASSERT_FALSE((node->predecessors.empty()));
    ASSERT_FALSE((node->successors.empty()));
    EXPECT_EQ(TestDomain::State(0), node->state);
    EXPECT_EQ(TestDomain::State(1), node->successors.front()->state);
    EXPECT_EQ(TestDomain::State(1), node->predecessors.front()->state);

    TestDomain::StateActionState updated_action({0},{2*TestDomain::costFactor()},{1});
    domain.updateAction(updated_action);

    int pred_count = node->predecessors.size();
    EXPECT_EQ(1 ,pred_count);

    int succ_count = node->successors.size();
    EXPECT_EQ(1 ,succ_count);

    EXPECT_EQ(TestDomain::State(1), node->successors.front()->state);
    EXPECT_EQ(TestDomain::State(1), node->predecessors.front()->state);
    
}

}
