/* heuristic_search library
 *
 * Copyright (c) 2016, 
 * Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>, 
 * Warsaw University of Technology.
 * All rights reserved.
 *  
 */

#include <gtest/gtest.h>

#include "heuristic_search/StdOpenList.h"
#include "heuristic_search/StdSearchSpace.h"
#include "heuristic_search/SearchingAlgorithm.h"
#include "heuristic_search/HeuristicSearch.h"
#include "heuristic_search/MPGAAStar.h"
#include "heuristic_search/DStarMain.h"

#include "search_domains/movingai_benchmark/Domain.h"

namespace test_movingai_benchmark{

typedef heuristic_search::SearchAlgorithmBegin<            
            heuristic_search::DStarMain<
            heuristic_search::MPGAAStar<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > >  MPGAAStarHog2TestAlgorithm_t;

    
TEST(test_MPGAAStarHog2, search_reinitialize_start)
{
    using movingai_benchmark::Domain;
    typedef Domain::State State;
    
    int W = 5;
    int H = 5;
    
    unsigned char map[] = {
            '.','.','.','.','.',
            '.','.','.','.','.',
            '.','.','.','.','.',
            '.','.','.','.','.',
            '.','.','.','.','.'
        };
    
    unsigned char unknown_map[] = {
            '.','.','.','.','.',
            '.','.','.','.','.',
            '.','.','T','T','.',
            '.','.','.','T','.',
            '.','.','.','T','.'
        };
    
    int observation_range = 2;
    
    Domain domain(&map[0], &unknown_map[0], W, H, observation_range);
    
    MPGAAStarHog2TestAlgorithm_t::Algorithm_t algorithm(domain, 
            heuristic_search::SearchingDirection::Forward);
    
    State start_state(1,4);
    State goal_state(4,4);
    
    algorithm.main(start_state, goal_state);
    
    EXPECT_EQ(goal_state, algorithm.start_node->state);
}

}//namespace test_movingai_benchmark
