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


#ifndef HEURISTIC_SEARCH_ASTAR_H
#define HEURISTIC_SEARCH_ASTAR_H

#include <cassert>

#include "heuristic_search/SearchLoop.h"
#include "heuristic_search/StdOpenList.h"
#include "heuristic_search/StdSearchSpace.h"
#include "heuristic_search/HeuristicSearch.h"
#include "heuristic_search/SearchingAlgorithm.h"

namespace heuristic_search{

template<typename B>
class AStar : public B 
{
public:
    typedef typename B::Domain_t::Cost Cost;
    typedef typename B::Key_t Key_t;
    
    template<typename final_t>
    class Node : public B::template Node<final_t>
    {
    public:
        typedef typename B::template Node<final_t> base_t;
        typedef typename final_t::Domain_t::State State_t;
        typedef typename final_t::Domain_t::Cost Cost_t;
        typedef typename final_t::Key_t Key_t;
        typedef typename final_t::Node_t Node_t;
        
        Node()
        {
            parent = nullptr;
        }
        
        virtual ~Node()
        {
            
        }
        
        void resetSearchData()
        {
            parent = nullptr;
            base_t::resetSearchData();
        }
        
        Node_t *parent;
    };
    
    
    template<typename final_t> 
    class Algorithm : public B::template Algorithm<final_t>
    {
    public:
        typedef typename B::template Algorithm<final_t> base_t;
    
        typedef typename final_t::Node_t Node_t;
        typedef typename final_t::Key_t Key_t;
        typedef typename final_t::Domain_t Domain_t;
        typedef typename final_t::Domain_t::State State;
        typedef typename final_t::Domain_t::Action Action;

        std::string description()
        {
            return "A*|" + base_t::description();
        }
        
        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {            
            reopen_count = 0;
            this_final = static_cast<typename final_t::Algorithm_t *>(this);
        }
        
        
        void initialize(State start, State goal)
        {                                
            if(base_t::searching_direction==SearchingDirection::Forward)
            {
                base_t::initialize(start, goal);
            }
            else
            {
                base_t::initialize(goal, start);
            }            
            
            if(base_t::start_node && base_t::goal_node)
            {
                
                base_t::start_node->visited = true;
                
                base_t::start_node->g = 0;
                Cost h =  this_final->heuristic(  
                                            base_t::start_node->state, 
                                            base_t::goal_node->state); 

                this_final->pushOpen(base_t::start_node, 
                        this_final->calculateKey(base_t::start_node, h));
            }
            
        }
        
        bool solutionFound()
        {
            return base_t::goal_node == base_t::open_list->topOpen();
        }
        
        void searchStep()
        {                    
            Node_t *node = base_t::open_list->topOpen();            
            base_t::open_list->popOpen();
            
            assert(!Domain_t::equal(node->g, Domain_t::costMax()));
            
            this_final->searchExpansionStep(node);        
        }
        
        auto neighbors(Node_t *node) -> decltype(
                    (base_t::searching_direction==SearchingDirection::Forward)?
                    base_t::search_space->successors(node):
                    base_t::search_space->predecessors(node)) 
        {
            return (base_t::searching_direction==SearchingDirection::Forward)?
                    base_t::search_space->successors(node):
                    base_t::search_space->predecessors(node);
        }
        
        auto neighbors_inv(Node_t *node) -> decltype(
                    (base_t::searching_direction==SearchingDirection::Forward)?
                    base_t::search_space->predecessors(node):
                    base_t::search_space->successors(node)) 
        {
            return (base_t::searching_direction==SearchingDirection::Forward)?
                    base_t::search_space->predecessors(node):
                    base_t::search_space->successors(node);
        }

        void searchExpansionStep(Node_t *node)
        {                                  
            Cost g,h;
            
            auto &neighbor_edges = neighbors(node);
                

            for(auto & neighbournode : neighbor_edges)
            {      

                if(!neighbournode)
                    continue;
                
                Cost action_cost = 
                        (base_t::searching_direction==SearchingDirection::Forward)?
                        this_final->cost(node->state, neighbournode->state):
                        this_final->cost(neighbournode->state, node->state);

                                
                if(Domain_t::equal(action_cost, Domain_t::costMax()) 
                        || Domain_t::equal(node->g, final_t::Domain_t::costMax()))
                {
                    g = final_t::Domain_t::costMax();
                }
                else
                {
                    g = node->g + action_cost;
                }
                
                assert(g > node->g || Domain_t::equal(g, Domain_t::costMax()));

                if(!neighbournode->visited || (g < neighbournode->g && !Domain_t::equal(g,neighbournode->g)))
                {                         
                    
                    if(!neighbournode->visited)
                    {
                        neighbournode->visited = true;
                    }
                    else
                    {
                        reopen_count++;
                    }
                    
                    neighbournode->g = g;
                    neighbournode->parent = node;
                    
                    if(Domain_t::equal(g, final_t::Domain_t::costMax()))
                    {
                        h = final_t::Domain_t::costMax();
                    }
                    else
                    {
                        h = base_t::heuristic(neighbournode->state, base_t::goal_node->state);
                    }
                    
                    Key_t key = this_final->calculateKey(neighbournode, h);
                    
                    this_final->pushOpen(neighbournode, key);
                }
            }       
        }
        
        int reopen_count;
        
        std::vector<State> getStatePath()
        {
            std::vector<State> path;

            if(!base_t::goal_node || !base_t::goal_node->visited)
                return path;
            
            return getStatePath(base_t::goal_node);
        }
        
        std::vector<State> getStatePath(Node_t * node_m)
        {            
            std::vector<State> path;
            
            while(node_m)
            {            
                if(base_t::searching_direction==SearchingDirection::Forward)
                {
                    path.insert(path.begin(),node_m->state); 
                }
                else
                {
                    path.push_back(node_m->state);
                }
                
                
                node_m = node_m->parent;                               
            }

            return path;

        }
                
    private:
        typename final_t::Algorithm_t *this_final;
    
    };
    
};

template<typename Domain>
using AStar_t = typename SearchAlgorithmBegin<            
            SearchLoop<
            AStar<
            HeuristicSearch<
            StdOpenList<
            StdSearchSpace<
        SearchAlgorithmEnd<Domain> > > > > > >::Algorithm_t;

}//namespace heuristic_search

#endif /* HEURISTIC_SEARCH_ASTAR_H */

