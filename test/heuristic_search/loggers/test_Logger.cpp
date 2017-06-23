/* heuristic_search library
 *
 * Copyright (c) 2016, 
 * Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>, 
 * Warsaw University of Technology.
 * All rights reserved.
 *  
 */


#include <gtest/gtest.h>

#include "heuristic_search/HeuristicSearch.h"
#include "heuristic_search/loggers/DStarLogger.h"
#include "heuristic_search/SearchLoop.h"
#include "heuristic_search/StdOpenList.h"
#include "heuristic_search/StdSearchSpace.h"
#include "heuristic_search/SearchingAlgorithm.h"
#include "heuristic_search/AStar.h"
#include "heuristic_search/DStarMain.h"
#include "heuristic_search/DStarExtraLite.h"
#include "heuristic_search/IncrementalSearch.h"

#include "../../heuristic_search/test_TestDomain.h"

namespace test_heuristic_search{

    
TEST(test_Logger, astar)
{        
    TestDomain domain;

    
    typedef heuristic_search::SearchAlgorithmBegin<            
            heuristic_search::loggers::Logger<
            heuristic_search::SearchLoop<
            heuristic_search::AStar<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<TestDomain> > > > > > > > AStarLoggerTestAlgorithm_t;
    
    AStarLoggerTestAlgorithm_t::Algorithm_t algorithm(domain);
       
    ASSERT_TRUE(algorithm.search({0},{5}));
    
    EXPECT_TRUE(algorithm.log.success);
    EXPECT_GT(algorithm.log.search_steps, 0);
    EXPECT_GT(algorithm.log.heap_counter, 0);
    EXPECT_GT(algorithm.log.heap_counter, 0);
    EXPECT_EQ(5*TestDomain::costFactor(), algorithm.log.path_cost*TestDomain::costFactor());

}
    
TEST(test_Logger, dstar)
{        
    TestDomain domain;

    
    typedef heuristic_search::SearchAlgorithmBegin< 
            heuristic_search::loggers::DStarLogger<
            heuristic_search::DStarMain<
            heuristic_search::SearchLoop<
            heuristic_search::DStarExtraLite<
            heuristic_search::IncrementalSearch_SearchStep<
            heuristic_search::AStar<
            heuristic_search::IncrementalSearch_Initialize<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<TestDomain> > > > > > > > > > > > DStarLoggerTestAlgorithm_t;
    
    DStarLoggerTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward, true);
       
    ASSERT_TRUE(algorithm.search({0},{5}));
    
    EXPECT_GT(algorithm.log.search_steps, 0);
    EXPECT_GT(algorithm.log.heap_counter, 0);
    
    heuristic_search::loggers::Log prev_log = algorithm.log;
    
    std::vector<std::pair<TestDomain::StateActionState, TestDomain::Cost> > updated_actions;
    updated_actions.push_back(
        std::make_pair(TestDomain::StateActionState(
                                {2},{10*TestDomain::costFactor()},{3}),1*TestDomain::costFactor()));
    domain.updateAction(TestDomain::StateActionState(
                                {2},{10*TestDomain::costFactor()},{3}));                            
                                
    algorithm.reinitialize({1}, updated_actions);
    ASSERT_TRUE(algorithm.search());    
    
    EXPECT_GT(algorithm.log.search_steps, prev_log.search_steps);
    EXPECT_GT(algorithm.log.heap_counter, prev_log.heap_counter);

}
    
TEST(test_Logger, dstar_2)
{        
    TestDomain domain;

    
    typedef heuristic_search::SearchAlgorithmBegin< 
            heuristic_search::loggers::DStarLogger<
            heuristic_search::DStarMain<
            heuristic_search::SearchLoop<
            heuristic_search::DStarExtraLite<
            heuristic_search::IncrementalSearch_SearchStep<
            heuristic_search::AStar<
            heuristic_search::IncrementalSearch_Initialize<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<TestDomain> > > > > > > > > > > > DStarLoggerTestAlgorithm_t;
    
    DStarLoggerTestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Backward);
       
    algorithm.main({0},{5});
    
    EXPECT_TRUE(algorithm.log.success);
    EXPECT_GT(algorithm.log.search_steps, 0);
    EXPECT_GT(algorithm.log.heap_counter, 0);
    
    EXPECT_EQ(5*TestDomain::costFactor(), algorithm.log.path_cost*TestDomain::costFactor());

}

    
}
