/* heuristic_search library
 *
 * Copyright (c) 2016, 
 * Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>, 
 * Warsaw University of Technology.
 * All rights reserved.
 *  
 */

#include <gtest/gtest.h>

#include <tuple>

#include "heuristic_search/DStarMain.h"
#include "heuristic_search/SearchingAlgorithm.h"

#include "../heuristic_search/test_TestDomain.h"

namespace test_heuristic_search{

template<typename B>
class DStarMainBase : public B
{
public:
    
    template<typename final_t> 
    class Algorithm : public B::template Algorithm<final_t>
    {
    public:    
        typedef typename final_t::Domain_t::State State_t;
        typedef typename final_t::Domain_t::Cost Cost_t;
        typedef typename final_t::Domain_t::StateActionState StateActionState_t;
        
        Algorithm()
        {
            path = {{1}, {2}, {3}, {4}, {5}};
            
            domain = new TestDomain();
            
            finished = false;
            found = false;
        }
        
        ~Algorithm()
        {
            delete domain;
        }

        State_t actionSelection()
        {
            return path[start.id];
        }

        void initialize(State_t _start, State_t _goal)
        {
            start = _start;
            goal = _goal;
        }

        void reinitialize(State_t _start, std::vector<std::pair<StateActionState_t, Cost_t> > const&updated_actions)
        {
            start = _start;

        }

        bool search()
        {
            finished = true;
            found = true;
            
            return found;
        }
        
        Cost_t cost(State_t, State_t)
        {
            return 1*TestDomain::costFactor();
        }

        typename final_t::Domain_t *domain;
        State_t start;
        State_t goal;
        State_t last_start;

        bool finished;
        bool found;
        
        std::vector<State_t> path;
    
    };
};

template<typename B>
using DStarMainTest=heuristic_search::DStarMain<DStarMainBase<B> >;

TEST(test_DStarMain, test1)
{
    using namespace heuristic_search;
    
    typedef SearchAlgorithmBegin<
                    DStarMainTest<
            SearchAlgorithmEnd<TestDomain> > >::Algorithm_t Algorithm_t;
    
    Algorithm_t algorithm;
    
    Algorithm_t::State start = {1};
    Algorithm_t::State goal = {5};
    
    algorithm.main(start,goal);
    
    EXPECT_EQ(algorithm.goal, algorithm.start);
    EXPECT_EQ(4*TestDomain::costFactor(), algorithm.travelled_path_cost);
    
}

}
