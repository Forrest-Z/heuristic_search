/* heuristic_search library
 *
 * Copyright (c) 2016, 
 * Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>, 
 * Warsaw University of Technology.
 * All rights reserved.
 *  
 */

#include <gtest/gtest.h>

#include "search_domains/movingai_benchmark/Domain.h"

#include "heuristic_search/StdSearchSpace.h"
#include "heuristic_search/SearchingAlgorithm.h"

namespace test_movingai_benchmark{

   
TEST(test_Domain, uninitialized)
{    
    unsigned char map[100];
    movingai_benchmark::Domain domain(&map[0], nullptr, 0, 0, 0);
    
    using movingai_benchmark::Domain;
    typedef movingai_benchmark::Domain::State State;
    
    EXPECT_EQ(0, domain.size());
    
    EXPECT_FALSE(domain.inRange(State(0,0)));
    
    EXPECT_EQ(-1, domain.stateId(State(0,0)));
    
    EXPECT_EQ(Domain::costMax(), domain.cost(State(0,0), State(1,1)));
   
}
   
TEST(test_Domain, in_range)
{    
    unsigned char map[100];
    movingai_benchmark::Domain domain(&map[0], nullptr, 10, 10, 0);
    using movingai_benchmark::Domain;
    typedef movingai_benchmark::Domain::State State;
    
    EXPECT_TRUE(domain.inRange(State(0,0)));
    EXPECT_FALSE(domain.inRange(State(-1,0)));
    EXPECT_FALSE(domain.inRange(State(-1,-1)));
    EXPECT_FALSE(domain.inRange(State(10,0)));
    EXPECT_FALSE(domain.inRange(State(10,10)));
    EXPECT_FALSE(domain.inRange(State(0,-1)));
    EXPECT_FALSE(domain.inRange(State(0,10)));
    EXPECT_FALSE(domain.inRange(State(0,10)));
    
   
}
   
TEST(test_Domain, stateId)
{    
    unsigned char map[100];
    movingai_benchmark::Domain domain(&map[0], nullptr, 10, 10, 0);
    using movingai_benchmark::Domain;
    typedef movingai_benchmark::Domain::State State;
    
    EXPECT_EQ(-1, domain.stateId(State(-1,-1)));
    EXPECT_EQ(-1, domain.stateId(State(10,10)));
    EXPECT_EQ(0, domain.stateId(State(0,0)));
    EXPECT_EQ(1, domain.stateId(State(1,0)));
    EXPECT_EQ(10, domain.stateId(State(0,1)));
    EXPECT_EQ(11, domain.stateId(State(1,1)));
    
   
}

TEST(test_Domain, equal)
{    
    using movingai_benchmark::Domain;
    
    EXPECT_TRUE(Domain::equal(Domain::costMax(), Domain::costMax()));
            
    EXPECT_FALSE(Domain::equal(0, Domain::costMax()));
    
    EXPECT_FALSE(Domain::equal(Domain::costMax() + Domain::costMax(), Domain::costMax()));
}


TEST(test_Domain, predecessors)
{    
    using movingai_benchmark::Domain;
    typedef Domain::State State;
    
    int W = 3;
    int H = 3;
    unsigned char map[] = {
            '.', '.', '.',
            '.', '.', '.',
            '.', '.', '.'
        };
    
    Domain domain(&map[0], nullptr, W, H, 0);
    
    auto predecessors = domain.getPredecessors(State(1,1));    
    ASSERT_EQ(8, predecessors.size());
    
    predecessors = domain.getPredecessors(State(0,0));    
    ASSERT_EQ(3, predecessors.size());
    
    predecessors = domain.getPredecessors(State(2,2));    
    ASSERT_EQ(3, predecessors.size());
    
    predecessors = domain.getPredecessors(State(0,1));    
    ASSERT_EQ(5, predecessors.size());
    
    predecessors = domain.getPredecessors(State(2,1));    
    ASSERT_EQ(5, predecessors.size());
    
    predecessors = domain.getPredecessors(State(1,0));    
    ASSERT_EQ(5, predecessors.size());
    
    predecessors = domain.getPredecessors(State(1,2));    
    ASSERT_EQ(5, predecessors.size());
    
}

TEST(test_Domain, successors)
{    
    using movingai_benchmark::Domain;
    typedef Domain::State State;
    
    int W = 3;
    int H = 3;
    unsigned char map[] = {
            '.', '.', '.',
            '.', '.', '.',
            '.', '.', '.'
        };
    
    Domain domain(&map[0], nullptr, W, H, 0);
    
    auto successors = domain.getSuccessors(State(1,1));    
    ASSERT_EQ(8, successors.size());
    
    successors = domain.getSuccessors(State(0,0));    
    ASSERT_EQ(3, successors.size());
    
    successors = domain.getSuccessors(State(2,2));    
    ASSERT_EQ(3, successors.size());
    
    successors = domain.getSuccessors(State(0,1));    
    ASSERT_EQ(5, successors.size());
    
    successors = domain.getSuccessors(State(2,1));    
    ASSERT_EQ(5, successors.size());
    
    successors = domain.getSuccessors(State(1,0));    
    ASSERT_EQ(5, successors.size());
    
    successors = domain.getSuccessors(State(1,2));    
    ASSERT_EQ(5, successors.size());
    
}

TEST(test_Domain, getActions)
{    
    using movingai_benchmark::Domain;
    
    int W = 2;
    int H = 2;
    unsigned char map[] = {
            '.', '.', 
            '.', '.'
        };
    
    Domain domain(&map[0], nullptr, W, H, 0);
    
    auto actions = domain.getActions();
    
    EXPECT_EQ(12, actions.size());
    
}

TEST(test_Domain, cost)
{    
    using movingai_benchmark::Domain;
    typedef Domain::State State;
    
    int W = 3;
    int H = 8;
    unsigned char map[] = {
            '.', '.', '.',
            '.', '.', '.',
            '.', 'G', '.',
            '.', '@', '.',
            '.', 'O', '.',
            '.', 'T', '.',
            '.', 'S', 'S',
            '.', 'W', 'W'
        };
    
    Domain domain(&map[0], nullptr, W, H, 0);
    
    //. - passable terrain
    //G - passable terrain
    //@ - out of bounds
    //O - out of bounds
    //T - trees (unpassable)
    //S - swamp (passable from regular terrain)
    //W - water (traversable, but not passable from terrain)
    
    EXPECT_EQ(1*Domain::costFactor(), domain.cost(State(0,0), State(1,0)));
    EXPECT_EQ(1*Domain::costFactor(), domain.cost(State(0,0), State(0,1)));
    EXPECT_NEAR(Domain::Cost(M_SQRT2*Domain::costFactor()), domain.cost(State(0,0), State(1,1)), Domain::costFactor()*0.001);
    
    EXPECT_EQ(Domain::costMax(), domain.cost(State(0,3), State(1,3)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(1,3), State(2,3)));
    
    EXPECT_EQ(Domain::costMax(), domain.cost(State(0,4), State(1,4)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(1,4), State(2,4)));
        
    EXPECT_EQ(Domain::costMax(), domain.cost(State(0,5), State(1,5)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(1,5), State(2,5)));
        
    EXPECT_EQ(1*Domain::costFactor(), domain.cost(State(0,6), State(1,6)));
    EXPECT_EQ(1*Domain::costFactor(), domain.cost(State(1,6), State(2,6)));
    EXPECT_EQ(1*Domain::costFactor(), domain.cost(State(1,6), State(1,7)));
    EXPECT_EQ(1*Domain::costFactor(), domain.cost(State(1,7), State(1,6)));
    
    EXPECT_EQ(Domain::costMax(), domain.cost(State(0,7), State(1,7)));
    EXPECT_EQ(1*Domain::costFactor(), domain.cost(State(1,7), State(2,7)));
}

TEST(test_Domain, cost_corner_0)
{    
    using movingai_benchmark::Domain;
    typedef Domain::State State;
    
    int W = 3;
    int H = 3;
    unsigned char map[] = {
            '.', '.', '.',
            '.', 'T', '.',
            '.', '.', '.'
        };
    
    Domain domain(&map[0], nullptr, W, H, 0);
        
    EXPECT_EQ(Domain::costMax(), domain.cost(State(0,1), State(1,0)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(1,0), State(2,1)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(2,1), State(1,2)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(1,2), State(0,1)));
}

TEST(test_Domain, cost_corner_1)
{    
    using movingai_benchmark::Domain;
    typedef Domain::State State;
    
    int W = 2;
    int H = 2;
    unsigned char map[] = {
            '.', 'T',
            'T', '.'
        };
    
    Domain domain(&map[0], nullptr, W, H, 0);
        
    EXPECT_EQ(Domain::costMax(), domain.cost(State(0,0), State(1,1)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(1,1), State(0,0)));
}

TEST(test_Domain, cost_corner_2)
{    
    using movingai_benchmark::Domain;
    typedef Domain::State State;
    
    int W = 2;
    int H = 2;
    unsigned char map[] = {
            'T', '.',
            '.', 'T'
        };
    
    Domain domain(&map[0], nullptr, W, H, 0);
        
    EXPECT_EQ(Domain::costMax(), domain.cost(State(0,1), State(1,0)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(1,0), State(0,1)));
}

TEST(test_Domain, cost_corner_3)
{    
    using movingai_benchmark::Domain;
    typedef Domain::State State;
    
    int W = 4;
    int H = 5;
    unsigned char map[] = {
            '.', '.','.','.',
            '.', '.','T','T',
            '.', 'T','.','.',
            '.', 'T','.','.',
            '.', 'T','.','.'
        };
    
    Domain domain(&map[0], nullptr, W, H, 0);
        
    EXPECT_EQ(Domain::costMax(), domain.cost(State(2,2), State(1,1)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(1,1), State(2,2)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(2,2), State(2,1)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(2,2), State(1,2)));
}

TEST(test_Domain, cost_corner_4)
{    
    using movingai_benchmark::Domain;
    typedef Domain::State State;
    
    int W = 4;
    int H = 5;
    unsigned char map[] = {
            '.', 'T','.','.',
            '.', 'T','.','.',
            '.', 'T','.','.',
            '.', '.','T','T',
            '.', '.','.','.'
        };
    
    Domain domain(&map[0], nullptr, W, H, 0);
        
    EXPECT_EQ(Domain::costMax(), domain.cost(State(2,2), State(1,3)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(1,3), State(2,2)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(2,2), State(1,2)));
    EXPECT_EQ(Domain::costMax(), domain.cost(State(2,2), State(2,3)));
}


TEST(test_Domain, mapUpdate_increase)
{  
    using movingai_benchmark::Domain;
    typedef Domain::State State;
    typedef Domain::Cost Cost;
    typedef Domain::StateActionState StateActionState;
    
    int W = 4;
    int H = 4;
    unsigned char map[] = {
            '.','.','.','.',
            '.','.','.','.',
            '.','.','.','.',
            '.','.','.','.'
        };
    
    unsigned char unknown_map[] = {
            '.','.','.','T',
            '.','T','.','.',
            '.','.','.','.',
            'T','.','.','.'
        };
    
    int observation_range = 2;
    
    Domain domain(&map[0], &unknown_map[0], W, H, observation_range);
    
    auto changed_actions = domain.mapUpdate(State(0,0));
    
    EXPECT_EQ(24, changed_actions.size());
    
    EXPECT_NE(changed_actions.end(), 
            std::find_if(changed_actions.begin(), changed_actions.end(), 
            [](std::pair<StateActionState, Cost> const &sas)->bool
            {
                return sas.first.from == State(1,1) 
                        && sas.first.to == State(0,0)
                        && sas.first.action.cost == Domain::costMax();
            }));
    
    EXPECT_NE(changed_actions.end(), 
            std::find_if(changed_actions.begin(), changed_actions.end(), 
            [](std::pair<StateActionState, Cost> const &sas)->bool
            {
                return sas.first.from == State(0,0) 
                        && sas.first.to == State(1,1)
                        && sas.first.action.cost == Domain::costMax();
            }));
    
    EXPECT_NE(changed_actions.end(), 
            std::find_if(changed_actions.begin(), changed_actions.end(), 
            [](std::pair<StateActionState, Cost> const &sas)->bool
            {
                return sas.first.from == State(0,1) 
                        && sas.first.to == State(1,0)
                        && sas.first.action.cost == Domain::costMax();
            }));
    
    EXPECT_NE(changed_actions.end(), 
            std::find_if(changed_actions.begin(), changed_actions.end(), 
            [](std::pair<StateActionState, Cost> const &sas)->bool
            {
                return sas.first.from == State(1,0) 
                        && sas.first.to == State(0,1)
                        && sas.first.action.cost == Domain::costMax();
            }));
            
    EXPECT_EQ(Domain::costMax(), domain.cost(State(0,0), State(1,1)));
    
}

TEST(test_Domain, mapUpdate_decrease)
{  
    using movingai_benchmark::Domain;
    typedef Domain::State State;
    typedef Domain::Cost Cost;
    typedef Domain::StateActionState StateActionState;
    
    int W = 4;
    int H = 4;
    
    unsigned char map[] = {
            '.','.','.','T',
            '.','T','.','.',
            '.','.','.','.',
            'T','.','.','.'
        };
    
    unsigned char unknown_map[] = {
            '.','.','.','.',
            '.','.','.','.',
            '.','.','.','.',
            '.','.','.','.'
        };
    
    int observation_range = 2;
    
    Domain domain(&map[0], &unknown_map[0], W, H, observation_range);
    
    auto changed_actions = domain.mapUpdate(State(0,0));
    
    EXPECT_EQ(24, changed_actions.size());
    
    EXPECT_NE(changed_actions.end(), 
            std::find_if(changed_actions.begin(), changed_actions.end(), 
            [](std::pair<StateActionState, Cost> const &sas)->bool
            {
                return sas.first.from == State(1,1) 
                        && sas.first.to == State(0,0)
                        && sas.first.action.cost != Domain::costMax();
            }));
    
    EXPECT_NE(changed_actions.end(), 
            std::find_if(changed_actions.begin(), changed_actions.end(), 
            [](std::pair<StateActionState, Cost> const &sas)->bool
            {
                return sas.first.from == State(0,0) 
                        && sas.first.to == State(1,1)
                        && sas.first.action.cost != Domain::costMax();
            }));
    
    EXPECT_NE(changed_actions.end(), 
            std::find_if(changed_actions.begin(), changed_actions.end(), 
            [](std::pair<StateActionState, Cost> const &sas)->bool
            {
                return sas.first.from == State(0,1) 
                        && sas.first.to == State(1,0)
                        && sas.first.action.cost != Domain::costMax();
            }));
    
    EXPECT_NE(changed_actions.end(), 
            std::find_if(changed_actions.begin(), changed_actions.end(), 
            [](std::pair<StateActionState, Cost> const &sas)->bool
            {
                return sas.first.from == State(1,0) 
                        && sas.first.to == State(0,1)
                        && sas.first.action.cost != Domain::costMax();
            }));
            
    EXPECT_NEAR(M_SQRT2*Domain::costFactor(), domain.cost(State(0,0), State(1,1)), 0.001*Domain::costFactor());
    
}

TEST(test_Domain, mapUpdate_search_space)
{  
    using movingai_benchmark::Domain;
    typedef Domain::State State;
//    typedef Domain::StateActionState StateActionState;
    
    int W = 4;
    int H = 4;
    
    unsigned char map[] = {
            '.','.','.','T',
            '.','T','.','.',
            '.','.','.','.',
            'T','.','.','.'
        };
    
    unsigned char unknown_map[] = {
            '.','.','.','.',
            '.','.','.','.',
            '.','.','.','.',
            '.','.','.','.'
        };
    
    int observation_range = 2;
    
    Domain domain(&map[0], &unknown_map[0], W, H, observation_range);
    
    
    typedef heuristic_search::SearchAlgorithmBegin<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<Domain> > > SearchingAlgorithm_t;
    SearchingAlgorithm_t::SearchSpace_t search_space(domain);
    SearchingAlgorithm_t::SearchSpace_t search_space_i(domain);    
    search_space_i.initialize();
    
    for(int i=0; i<W; ++i)
    {
        for(int j=0; j<H; j++)
        {
            State state(i,j);
            
            int ni=0,n=0;
            
            auto preds = search_space.predecessors(search_space.getNode(state));
            n+=preds.size();
            
            auto preds_i = search_space_i.predecessors(search_space.getNode(state));
            ni+=preds_i.size();
            
            auto succs = search_space.successors(search_space.getNode(state));
            n+=succs.size();
            
            auto succs_i = search_space_i.successors(search_space.getNode(state));
            ni+=succs_i.size();
            
            EXPECT_EQ(ni,n);
        }
    }
    
}

}
