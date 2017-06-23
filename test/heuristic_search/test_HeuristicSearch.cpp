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
#include "heuristic_search/StdOpenList.h"
#include "heuristic_search/HeuristicSearch.h"
#include "heuristic_search/SearchingAlgorithm.h"

#include "../heuristic_search/test_TestDomain.h"

namespace test_heuristic_search{


template<typename B>
class HeuristicSearchTestLayer : public B
{
public:
    typedef typename B::Domain_t::Cost Cost_t;
    typedef typename B::Domain_t::Cost Key_t;
    
    static Cost_t costMax()
    {
        return std::numeric_limits<Cost_t>::max();
    }
    
    static Key_t keyMax()
    {
        return std::numeric_limits<typename B::Domain_t::Cost>::max();
    }
    
    template<typename final_t>
    class Algorithm : public B::template Algorithm<final_t>
    {
    public:
        template<typename ...Args>
        Algorithm(Args&&... args) : B::template Algorithm<final_t>(std::forward<Args>(args)...)
        {
            
        }
    };
};

typedef heuristic_search::SearchAlgorithmBegin<
            HeuristicSearchTestLayer<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<TestDomain> > > > > > HeuristicSearchAlgorithm_t;
    
TEST(test_HeuristicSearch, uninitialized)
{    
    TestDomain domain;   
    
    HeuristicSearchAlgorithm_t::OpenList_t open_list;
    HeuristicSearchAlgorithm_t::SearchSpace_t search_space(domain);
    
    HeuristicSearchAlgorithm_t::Algorithm_t algorithm(domain, open_list, search_space);
    
    EXPECT_FALSE(algorithm.initialized);
    EXPECT_FALSE(algorithm.finished);
    EXPECT_FALSE(algorithm.found);
    EXPECT_EQ(nullptr, algorithm.start_node);
    EXPECT_EQ(nullptr, algorithm.goal_node);
       
    
}
    
TEST(test_HeuristicSearch, uninitialized2)
{    
    TestDomain domain;   
    
    HeuristicSearchAlgorithm_t::OpenList_t open_list;
    
    HeuristicSearchAlgorithm_t::Algorithm_t algorithm(domain, open_list);
    
    EXPECT_FALSE(algorithm.initialized);
    EXPECT_FALSE(algorithm.finished);
    EXPECT_FALSE(algorithm.found);
    EXPECT_EQ(nullptr, algorithm.start_node);
    EXPECT_EQ(nullptr, algorithm.goal_node);
       
    
}
    
TEST(test_HeuristicSearch, uninitialized3)
{    
    TestDomain domain;   
    
    HeuristicSearchAlgorithm_t::SearchSpace_t search_space(domain);
    
    HeuristicSearchAlgorithm_t::Algorithm_t algorithm(domain, search_space);
    
    EXPECT_FALSE(algorithm.initialized);
    EXPECT_FALSE(algorithm.finished);
    EXPECT_FALSE(algorithm.found);
    EXPECT_EQ(nullptr, algorithm.start_node);
    EXPECT_EQ(nullptr, algorithm.goal_node);
       
    
}
    
TEST(test_HeuristicSearch, uninitialized4)
{    
    TestDomain domain;   
    
    HeuristicSearchAlgorithm_t::Algorithm_t algorithm(domain);
    
    EXPECT_FALSE(algorithm.initialized);
    EXPECT_FALSE(algorithm.finished);
    EXPECT_FALSE(algorithm.found);
    EXPECT_EQ(nullptr, algorithm.start_node);
    EXPECT_EQ(nullptr, algorithm.goal_node);
       
    
}
    
TEST(test_HeuristicSearch, initialize)
{    
    TestDomain domain;   
    
    HeuristicSearchAlgorithm_t::Algorithm_t algorithm(domain);
    
    algorithm.initialize({0}, {5});
    
    EXPECT_TRUE(algorithm.initialized);
    EXPECT_FALSE(algorithm.finished);
    EXPECT_FALSE(algorithm.found);
    EXPECT_NE(nullptr, algorithm.start_node);
    EXPECT_NE(nullptr, algorithm.goal_node);
       
    
}

}
