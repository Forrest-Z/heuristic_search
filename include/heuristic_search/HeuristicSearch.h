/* By downloading, copying, installing or using the software you agree to this license.
 * 
 *                           License Agreement
 *                      For heuristic_search library
 *
 * Copyright (c) 2016, 
 * Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>, 
 * Warsaw University of Technology.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holders nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */


#ifndef HEURISTIC_SEARCH_HEURISTICSEARCH_H
#define HEURISTIC_SEARCH_HEURISTICSEARCH_H

#include <list>
#include <limits>
#include <sstream>
#include <vector>
#include <cassert>

namespace heuristic_search{


enum class SearchingDirection
{
    Forward,
    Backward    
};

template<typename B>
class HeuristicSearch : public B
{
public: 
           
    typedef B HeuristicSearch_base_t;
    
    typedef typename B::Domain_t::Cost Cost;
    typedef Cost Key_t;
    
    
    static Key_t keyMax()
    {
        return B::Domain_t::costMax();
    }
    
    static bool equal(Key_t const& key1, Key_t const& key2)
    {
        return B::Domain_t::equal(key1, key2);
    }
    
    
    template<typename final_t>
    class Node : public B::template Node<final_t>
    {
    public:
        typedef typename final_t::Domain_t::Cost Cost_t;
        typedef typename B::template Node<final_t> base_t;
        
        Node()
        {
            g = final_t::Domain_t::costMax();
            
            visited = false;
        }
        
        virtual ~Node()
        {
            
        }
        
        void resetSearchData()
        {
            g = final_t::Domain_t::costMax();
            
            visited = false;
            
            base_t::resetSearchData();
        }
        
        
        Cost_t g;
        
        bool visited;                
    };
    
    
    
    template<typename final_t> 
    class Algorithm : public B::template Algorithm<final_t>
    {
    public:
        typedef typename final_t::Domain_t Domain_t;
        typedef typename final_t::OpenList_t OpenList_t;
        typedef typename final_t::Node_t Node_t;
        typedef typename final_t::SearchSpace_t SearchSpace_t;
        
        typedef typename Domain_t::State State_t;
        typedef typename Domain_t::Cost Cost_t;
        
        std::string description()
        {
            std::stringstream desc;
            desc
                    << ((searching_direction==SearchingDirection::Forward)?"Fwd":"Bckwd")
                    << "|"
                    << ((search_space_initialized)?"preinit":"uninit")
                    << "|"
                    << ((search_space_undirected)?"undir":"dir")
                    << "|"
                    << domain->description();
            return desc.str();
        }
        
        Algorithm(Domain_t &_domain, 
                SearchingDirection _searching_direction = SearchingDirection::Forward,
                bool initialize_search_space = false, bool undirected = false) 
            : B::template Algorithm<final_t>()
        {
            domain = &_domain;
            open_list = new OpenList_t;
            search_space = new SearchSpace_t(_domain, undirected);
            start_node = nullptr;
            goal_node = nullptr;
            
            initialized = false;
            finished = false;
            found = false;
            
            owns_open_list = true;
            owns_search_space = true;
            
            searching_direction = _searching_direction;
            search_space_undirected = undirected;
            
            if(initialize_search_space)
            {
                search_space->initialize();
                search_space_initialized = true;
            }
            else
            {
                search_space_initialized = false;
            }
            
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
        }
        
        Algorithm(Domain_t &_domain, SearchSpace_t &_search_space, 
                SearchingDirection _searching_direction = SearchingDirection::Forward,
                bool initialize_search_space = false) 
            : B::template Algorithm<final_t>()
        {
            domain = &_domain;
            open_list = new OpenList_t;
            search_space = &_search_space;
            start_node = nullptr;
            goal_node = nullptr;
            
            initialized = false;
            finished = false;
            found = false;
            
            owns_open_list = true;
            owns_search_space = false;
            
            searching_direction = _searching_direction;
            
            if(initialize_search_space)
            {
                search_space->initialize();
                search_space_initialized = true;
            }
            else
            {
                search_space_initialized = false;
            }
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
        }
        
        Algorithm(Domain_t &_domain, OpenList_t &_open_list, 
                SearchingDirection _searching_direction = SearchingDirection::Forward,
                bool initialize_search_space = false, bool undirected = false) 
            : B::template Algorithm<final_t>()
        {
            domain = &_domain;
            open_list = &_open_list;
            search_space = new SearchSpace_t(_domain, undirected);
            start_node = nullptr;
            goal_node = nullptr;
            
            initialized = false;
            finished = false;
            found = false;
            
            owns_open_list = false;
            owns_search_space = true;
            
            searching_direction = _searching_direction;
            search_space_undirected = undirected;
            
            if(initialize_search_space)
            {
                search_space->initialize();
                search_space_initialized = true;
            }
            else
            {
                search_space_initialized = false;
            }
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
        }
        
        Algorithm(Domain_t &_domain, OpenList_t &_open_list, SearchSpace_t &_search_space, 
                SearchingDirection _searching_direction = SearchingDirection::Forward,
                bool initialize_search_space = false) 
            : B::template Algorithm<final_t>()
        {
            domain = &_domain;
            open_list = &_open_list;
            search_space = &_search_space;
            start_node = nullptr;
            goal_node = nullptr;
            
            initialized = false;
            finished = false;
            found = false;
            
            owns_open_list = false;
            owns_search_space = false;
            
            searching_direction = _searching_direction;
            
            if(initialize_search_space)
            {
                search_space->initialize();
                search_space_initialized = true;
            }
            else
            {
                search_space_initialized = false;
            }
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
        }
        
        virtual ~Algorithm()
        {
            if(owns_open_list)
            {
                delete open_list;
            }
            
            if(owns_search_space)
            {
                delete search_space;
            }
        }
        
        void initialize(State_t start, State_t goal)
        {
            if(initialized)
            {
                reset();
            }
            
            start_node = search_space->getNode(start);
            goal_node = search_space->getNode(goal);
            
            if(start_node && goal_node)
            {
                initialized = true;
                
                goal_node->state = goal;
                
                start_node->state = start;
            }
        }
        
        void reset()
        {
            start_node = nullptr;
            goal_node = nullptr;
            
            initialized = false;
            finished = false;
            found = false;
            
            search_space->resetSearchData();
            
            open_list->clear();
            
        }
        
        Cost_t basicHeuristic(State_t from, State_t to)
        {
            return domain->heuristic(from, to);
        }
        
        Cost_t heuristic(State_t from, State_t to)
        {
            return this_final->basicHeuristic(from, to);
        }
        
        Cost_t cost(State_t from, State_t to)
        {
            return domain->cost(from, to);
        }
        
        Key_t calculateKey(Node_t *node, Cost_t h)
        {
            if(Domain_t::equal(node->g,Domain_t::costMax()) 
                    || Domain_t::equal(h, Domain_t::costMax()))
            {
                return Domain_t::costMax();
            }
            
            Key_t key = node->g + h;
            
            assert(key>=0);
            
            return key;
        }
        
        static Key_t keyMax()
        {
            return HeuristicSearch<B>::keyMax();
        }
        
        static bool equal(Key_t const& key1, Key_t const& key2)
        {
            return HeuristicSearch<B>::equal(key1, key2);
        }
        
        void pushOpen(Node_t *node, typename final_t::Key_t key)
        {            
            open_list->pushOpen(node, key);
        }
        
        bool openListEmpty()
        {
            return open_list->empty() || 
                    final_t::equal(open_list->topKey(), final_t::keyMax());
        }
        
        bool search_space_initialized;
        bool search_space_undirected;
        SearchingDirection searching_direction;
        Domain_t *domain;
        OpenList_t *open_list;
        SearchSpace_t *search_space;
        Node_t *start_node;
        Node_t *goal_node;
        
        bool initialized;
        bool finished;
        bool found;
        
    private:
        bool owns_open_list;
        bool owns_search_space;
        
        typename final_t::Algorithm_t* this_final;
    };
    
};

}//namespace heuristic_search


#endif /* HEURISTIC_SEARCH_HEURISTICSEARCH_H */

