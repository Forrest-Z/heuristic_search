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


#ifndef MPGAASTAR_H
#define MPGAASTAR_H

#include <utility>
#include <string>
#include <vector>
#include <algorithm>
#include <cassert>

#include "heuristic_search/StdHeap.h"

namespace heuristic_search{


template<typename B>
class MPGAAStar : public B
{
public:             
    
    typedef typename B::Domain_t::Cost Cost;
    
    template<typename final_t> class HOpenListElement;
    
    template<typename final_t>
    class Node : public B::template Node<final_t>
    {
    public:
        typedef typename B::template Node<final_t> base_t;
        typedef typename final_t::Domain_t Domain_t;
        typedef typename Domain_t::Cost Cost_t;
        typedef typename final_t::Node_t Node_t;
        
        Node() : base_t()
        {
            h = Cost_t(0);
            h_initialized = false;
            search_stamp = 0;
            parent = nullptr;
            support = nullptr;
            next = nullptr;
            
            h_open_list_element = new HOpenListElement<final_t>(this);
        }
        
        virtual ~Node()
        {
            delete h_open_list_element;
        }
        
        void resetSearchData()
        {
            h = Cost_t(0);
            parent = nullptr;
            support = nullptr;
            next = nullptr;
            h_initialized = false;
            search_stamp = 0;
            base_t::resetSearchData();
        }
        
        Cost_t h;
    
        Node_t* parent;
        Node_t* support;
        Node_t* next;
        
        int search_stamp;

        bool h_initialized;
        
        HOpenListElement<final_t> *h_open_list_element;
    };
    
    template<typename final_t>
    class HOpenListElement : public StdHeapElement<Cost>
    {
    public:
        typedef Node<final_t> Node_t;
        typedef StdHeapElement<Cost> StdHeapElement_t;
        
        HOpenListElement(Node_t *node):
                StdHeapElement_t(final_t::Domain_t::costMax()),
                node(node)                
        {
            
        }
        
        Node_t *node;
        
    };
    
    template<typename final_t>
    class HOpenList :   public StdHeap<Cost>
    {
    public:        
        typedef StdHeap<Cost> StdHeap_t;
        typedef HOpenListElement<final_t> HOpenListElement_t;
        typedef typename final_t::Node_t Node_t;
        
        HOpenList() : StdHeap_t(final_t::Domain_t::costMax())
        {
            
        }
        
        Node_t *topOpen()
        {                        
            HOpenListElement_t *h_heap_element = static_cast<HOpenListElement_t*>(
                    StdHeap_t::topOpen());
            return static_cast<Node_t*>(h_heap_element->node);
        }
        
        void pushOpen(Node_t *v, Cost h)
        {
            StdHeap_t::pushOpen(v->h_open_list_element, h);
            
            v->h = v->h_open_list_element->key;
            v->h_initialized = true;
        }
    };
    
    template<typename final_t> 
    class Algorithm : public B::template Algorithm<final_t>
    {
    public:
        typedef typename B::template Algorithm<final_t> base_t;
        
        typedef typename final_t::Domain_t Domain_t;
        typedef typename final_t::OpenList_t OpenList_t;
        typedef typename final_t::Node_t Node_t;
        typedef typename final_t::SearchSpace_t SearchSpace_t;
        typedef typename final_t::Key_t Key_t;
        
        typedef typename Domain_t::State State_t;
        typedef typename Domain_t::StateActionState StateActionState_t;
        typedef typename Domain_t::Cost Cost_t;
        
        std::string description()
        {
            return "MPGAA*|" + base_t::description();
        }
        
        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {     
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
            search_counter = 0;
            travelled_path_cost = 0;
            h_open_list = new typename final_t::template HOpenList<final_t>();
        }
        
        virtual ~Algorithm()
        {
            delete h_open_list;
        }
                               
        void initialize(State_t start, State_t goal)
        {
            base_t::initialize(start, goal);
            
            search_counter = 0;
            travelled_path_cost = 0;
                                    
            if(base_t::start_node && base_t::goal_node)
            {                                
                base_t::start_node->visited = true;               
                base_t::start_node->g = 0;
                                
                Cost h =  heuristic(base_t::start_node); 

                this_final->pushOpen(base_t::start_node, 
                        this_final->calculateKey(base_t::start_node, h));
            }
        }
        
        void initializeState(Node_t *node)
        {
            if(node->search_stamp != search_counter)
            {
                node->search_stamp = search_counter;
                        
                node->g = Domain_t::costMax();
                node->key = base_t::keyMax();
                
                node->parent = nullptr;
                node->visited = false;
                                
                assert(node->open == false);
                assert(node->closed == false);
                assert(node->h_open_list_element->open == false);
            }            
        }
              
        bool goalCondition(Node_t *node)
        {                        
            while(node->next != nullptr
                && 
                Domain_t::equal(node->h, node->next->h + 
                    this_final->cost(node->state, node->next->state)))
            {
                node = node->next;
            }
            
            return node->state == base_t::goal_node->state;
        }
        
        Cost_t heuristic(Node_t *node)
        {
            if(!node->h_initialized)
            {
                node->h = base_t::heuristic(node->state, base_t::goal_node->state);
                node->h_initialized = true;
            }
            
            return node->h;
        }
                
        void searchExpansionStep(Node_t *node)
        {  
            Cost g,h;
            
            auto &successor_edges = base_t::search_space->successors(node);
                

            for(Node_t *neighbournode : successor_edges)
            {      
                if(!neighbournode)
                    continue;
                
                initializeState(neighbournode);
                
                Cost action_cost = this_final->cost(node->state, neighbournode->state);

                                
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
                    
                    neighbournode->g = g;
                    neighbournode->parent = node;
                    
                    h = heuristic(neighbournode);                    
                    
                    Key_t key = this_final->calculateKey(neighbournode, h);
                    
                    this_final->pushOpen(neighbournode, key);
                }
            }       
        }
        
                        
        void searchStep()
        {                    
            Node_t *node = base_t::open_list->topOpen();            
            base_t::open_list->popOpen();
            
            assert(node->closed);
            assert(!node->open);
            
            closed.push_back(node);
            
            this_final->searchExpansionStep(node);   
        }
        
        Node_t* aStar()
        {              
            assert(this_final->initialized);
            
            search_counter++;   
            
            this_final->found = false;
            this_final->finished = false;
            
            assert(base_t::start_node);
            
            base_t::open_list->clear();
            
            initializeState(base_t::start_node);
            
            base_t::start_node->visited = true;               
            base_t::start_node->g = 0;
                        
            assert(closed.empty());
          
            
            Cost h =  heuristic(base_t::start_node); 
            this_final->pushOpen(base_t::start_node, 
                    this_final->calculateKey(base_t::start_node, h));
            
            while(!base_t::openListEmpty())
            {
                Node_t *node =  base_t::open_list->topOpen(); 

                if(goalCondition(node))
                {
                    this_final->found = true;
                    this_final->finished = true;
                    return node;
                }
                
                this_final->searchStep();
            }
            
            this_final->finished = true;
            
            return nullptr;
            
        }
        
        void buildPath(Node_t *node)
        {
            while(node->state != base_t::start_node->state)
            {
                Node_t * parent_node = node->parent;                
                parent_node->next = node;                
                node = parent_node;
            }
        }
        
        void insertState(Node_t *node, Node_t *next_node)
        {
            assert(next_node);
            assert(next_node->h >= 0);
            assert(node != next_node);
            
            Cost_t c = this_final->cost(node->state, next_node->state);
            
            assert(c>=0);
            
            Cost_t h = Domain_t::costMax();
            
            if(!Domain_t::equal(c, Domain_t::costMax()) 
                    && !Domain_t::equal(next_node->h, Domain_t::costMax()))
            {
                h = c + next_node->h;
            }
            
            if(node->h > h && !Domain_t::equal(node->h, h))
            {
                node->h = h;
                
                node->next = nullptr;
                node->support = next_node;
                
                h_open_list->pushOpen(node, h);
            }
        }
        
        bool reestablishConsistencyStep()
        { 
            Node_t *node = h_open_list->topOpen();
                
            if(node==nullptr)
            {
                return false;
            }

            h_open_list->popOpen();

            if(node->support->next)
            {
                node->next = node->support;
            }

            auto &predecessors_ = base_t::search_space->predecessors(node);

            for(Node_t *neighbournode : predecessors_)
            {      

                if(neighbournode && neighbournode->visited)
                {
                    insertState(neighbournode, node);
                }
            }
            
            return true;
        }
        
        void reestablishConsistency()
        {                        
            while(!h_open_list->empty())
            {
                if(!this_final->reestablishConsistencyStep())
                    break;  
            }
        }
        
        void updateHeuristics(Node_t *found_node)
        {
            assert(!Domain_t::equal(Domain_t::costMax(), found_node->h));
            assert(!Domain_t::equal(Domain_t::costMax(), found_node->g));
            
            Cost_t found_f = found_node->h + found_node->g;
            
            for(Node_t *node : closed)
            {
                assert(node->visited);
                
                if(!node->closed)
                {
                    continue;
                }
                
                assert(!node->open);
                
                if(!Domain_t::equal(Domain_t::costMax(), node->g))
                {
                    assert(found_f - node->g >= 0);
                    node->h = found_f - node->g;
                }                
                
                node->closed = false;
            }
            
            closed.clear();
        }
        
        void reinitialize(std::vector<std::pair<StateActionState_t, Cost_t> > const& updated_actions)
        {                           
            h_open_list->clear();
            
            bool any_decreased = false;
            
            for(auto &updated_action_pair : updated_actions)
            {
                StateActionState_t const& updated_action = updated_action_pair.first;
                Cost_t const& old_cost = updated_action_pair.second;
                
                State_t const& from_state = updated_action.from;
                                
                Node_t *from_node = base_t::search_space->getNode(from_state);
                
                if(!from_node->visited)
                {
                    continue;
                }
                     
                State_t to_state = updated_action.to;
                Node_t * to_node = base_t::search_space->getNode(to_state);
                                                              
                Cost_t new_cost = updated_action.action.cost;
                
                
                if(old_cost > new_cost)
                {
                    insertState(from_node, to_node);
                    any_decreased = true;
                }
                else if(old_cost < new_cost)
                {
                    from_node->next = nullptr;
                }                
                
            }
            
            if(any_decreased)
            {
                reestablishConsistency();
            }
        }
                
        bool solutionFound()
        {
            if(base_t::initialized && base_t::found)
            {                      
                if(goalCondition(base_t::start_node))
                {
                    return true;
                }
            }
            
            return false;
        }
        
        bool search()
        {
            if(solutionFound())
            {
                return true;
            }
            
            Node_t* found_node = aStar();
                
            if(found_node == nullptr)
            {
                return false;
            }

            updateHeuristics(found_node);
            buildPath(found_node);
            
            return true;
        }

        bool search(State_t start, State_t goal)
        {
            this_final->initialize(start, goal);
            return this_final->search();
        }
        
        void reinitialize(State_t new_start, std::vector<std::pair<StateActionState_t, Cost_t> > const& changed_actions)
        {
            base_t::start_node = base_t::search_space->getNode(new_start);
            
            if(!changed_actions.empty())
            {                
                reinitialize(changed_actions);
            }    
        }
        
        State_t actionSelection()
        {                        
            assert(base_t::start_node);
            
            Node_t* next_node = base_t::start_node->next;
            
            if(next_node == nullptr)
                return base_t::start_node->state;
            
            assert(base_t::start_node->state != next_node->state);
            
            return next_node->state;
            
        }
                
        std::vector<State_t> getStatePath(Node_t * node)
        {
            std::vector<State_t> path;
            
            while(node)
            {            
                assert(node->visited);
                assert(!Domain_t::equal(node->g, Domain_t::costMax()));                
                assert(std::find(path.begin(), path.end(), node->state)==path.end());
                
                path.push_back(node->state);     
                
                if(node->state == base_t::goal_node->state)
                {
                    break;
                }
                                
                node = node->next;                               
            }

            return path;
        }
        
        std::vector<State_t> getStatePath()
        {
            return getStatePath(base_t::start_node);
        }
        
    protected:
        typename final_t::Algorithm_t* this_final;
        std::vector<Node_t*> closed;
        Cost_t travelled_path_cost;
        typename final_t::template HOpenList<final_t> *h_open_list;
        int search_counter;
    };
};

template<typename B>
class MPGAAStar_Main : public B
{
public:       
    template<typename final_t> 
    class Algorithm : public B::template Algorithm<final_t>
    {
    public:
        typedef typename B::template Algorithm<final_t> base_t;
        
        typedef typename final_t::Domain_t Domain_t;
        typedef typename final_t::OpenList_t OpenList_t;
        typedef typename final_t::Node_t Node_t;
        typedef typename final_t::SearchSpace_t SearchSpace_t;
        typedef typename final_t::Key_t Key_t;
        
        typedef typename Domain_t::State State_t;
        typedef typename Domain_t::StateActionState StateActionState_t;
        typedef typename Domain_t::Cost Cost_t;
                
        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {     
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
            travelled_path_cost = 0;
        }
        
        
        void main(State_t start, State_t goal)
        {            
            initialize(start, goal);
            
            base_t::domain->mapUpdate(start);
                                                
            while(base_t::start_node != base_t::goal_node)
            {                            
                Node_t* found_node = base_t::aStar();
                
                if(!found_node)
                {
                    break;
                }
                
                base_t::updateHeuristics(found_node);
                base_t::buildPath(found_node);
                
                std::vector<std::pair<StateActionState_t, Cost_t> > updated_actions;
                
                while(base_t::start_node != base_t::goal_node)
                {                                        
                    travelled_path_cost += base_t::start_node->next.action.cost;
                    
                    State_t new_start = base_t::start_node->next.neighbor->state;
                    base_t::start_node->next = nullptr;
                    
                    base_t::start_node = base_t::search_space->getNode(new_start);
                                        
                    updated_actions = base_t::domain->mapUpdate(start);
                                                            
                    if(!updated_actions.empty())
                    {
                        break;
                    }                    
                }       
                
                base_t::reinitialize(updated_actions);
            }   
        }
        
    protected:
        typename final_t::Algorithm_t* this_final;
        Cost_t travelled_path_cost;
        
    };
    
};

}//namespace heuristic_search

#endif /* MPGAASTAR_H */

