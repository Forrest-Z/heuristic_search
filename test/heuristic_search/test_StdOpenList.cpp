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
#include "heuristic_search/HeuristicSearch.h"
#include "heuristic_search/SearchingAlgorithm.h"

#include "../heuristic_search/test_TestDomain.h"

namespace test_heuristic_search{


template<typename B>
class StdOpenListTestLayer : public B
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
    
    static bool equal(Key_t const& key1, Key_t const& key2)
    {
        return key1 == key2;
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
            StdOpenListTestLayer<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
        heuristic_search::SearchAlgorithmEnd<TestDomain> > > > > StdOpenListAlgorithm_t;
    
TEST(test_StdOpenList, empty)
{        
    StdOpenListAlgorithm_t::OpenList_t open_list;
       
    EXPECT_TRUE(open_list.empty());
    
    open_list.popOpen();
    EXPECT_TRUE(open_list.empty());
    
    EXPECT_EQ(nullptr, open_list.topOpen());
    EXPECT_EQ(StdOpenListAlgorithm_t::keyMax(), open_list.topKey());
}
    
TEST(test_StdOpenList, push_top)
{        
    StdOpenListAlgorithm_t::OpenList_t open_list;
    
    StdOpenListAlgorithm_t::Node_t node1, node2;
           
    open_list.pushOpen(&node2, 2);
    open_list.pushOpen(&node1, 1);
       
    ASSERT_FALSE(open_list.empty());
    
    EXPECT_EQ(1, open_list.topKey());
    EXPECT_EQ(&node1, open_list.topOpen());
}
    
TEST(test_StdOpenList, push_pop_top)
{        
    StdOpenListAlgorithm_t::OpenList_t open_list;
    
    StdOpenListAlgorithm_t::Node_t node1, node2;
           
    open_list.pushOpen(&node2, 2);
    open_list.pushOpen(&node1, 1);
    
    open_list.popOpen();
       
    ASSERT_FALSE(open_list.empty());
    
    EXPECT_EQ(2, open_list.topKey());
    EXPECT_EQ(&node2, open_list.topOpen());
        
    open_list.popOpen();
    ASSERT_TRUE(open_list.empty());
}
    
TEST(test_StdOpenList, push_remove_top)
{        
    StdOpenListAlgorithm_t::OpenList_t open_list;
    
    StdOpenListAlgorithm_t::Node_t node1, node2;
           
    open_list.pushOpen(&node2, 2);
    open_list.pushOpen(&node1, 1);
    
    open_list.removeOpen(&node1);
       
    ASSERT_FALSE(open_list.empty());
    
    EXPECT_EQ(2, open_list.topKey());
    EXPECT_EQ(&node2, open_list.topOpen());
}
    
TEST(test_StdOpenList, push_push_top)
{        
    StdOpenListAlgorithm_t::OpenList_t open_list;
    
    StdOpenListAlgorithm_t::Node_t node1, node2;
           
    open_list.pushOpen(&node2, 2);
    open_list.pushOpen(&node1, 1);
    
    open_list.pushOpen(&node1, 3);
       
    ASSERT_FALSE(open_list.empty());
    
    EXPECT_EQ(2, open_list.topKey());
    EXPECT_EQ(&node2, open_list.topOpen());
}
    

}
