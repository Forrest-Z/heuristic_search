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


#ifndef DSTARLITECOMMON_H
#define DSTARLITECOMMON_H

#include <utility>
#include <string>
#include <vector>
#include <algorithm>
#include <cassert>

namespace heuristic_search{

template<typename B>
class DStarLiteCommon : public B
{
public:             
    
    typedef typename B::Domain_t::Cost Cost;
    typedef std::pair<typename B::Key_t, typename B::Domain_t::Cost> Key_t;
    
    
    static Key_t keyMax()
    {
        return Key_t(B::Domain_t::costMax(), B::Domain_t::costMax());
    }
    
    static bool equal(Key_t const& key1, Key_t const& key2)
    {
        return B::Domain_t::equal(key1.first, key2.first)
                && B::Domain_t::equal(key1.second, key2.second);
    }
    
    template<typename final_t>
    class Node : public B::template Node<final_t>
    {
    public:
        typedef typename B::template Node<final_t> base_t;
        typedef typename final_t::Domain_t Domain_t;
        typedef typename Domain_t::Cost Cost_t;
        
        Node()
        {
            rhs = Domain_t::costMax();
        }
        
        virtual ~Node()
        {
            
        }
        
        void resetSearchData()
        {
            rhs = Domain_t::costMax();
            base_t::resetSearchData();
        }
        
        Cost_t rhs;
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
        
        
        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {     
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
            k_m = 0;
            
            assert(SearchingDirection::Backward == base_t::searching_direction);
        }
        
        virtual ~Algorithm()
        {
            
        }
        
        Key_t calculateKey(Node_t *node, Cost_t h)
        {
            if(Domain_t::equal(node->g, Domain_t::costMax()) 
                    &&  Domain_t::equal(node->rhs, Domain_t::costMax()))
            {
                return keyMax();
            }
            
            if(Domain_t::equal(h, Domain_t::costMax()))
            {
                return Key_t(Domain_t::costMax(), std::min(node->g, node->rhs));
            }
            
            return Key_t(std::min(node->g, node->rhs) + h + k_m, std::min(node->g, node->rhs));
        }
        
        Key_t recalculateKey(Node_t *node)
        {     
            Cost h =  this_final->heuristic(  
                                            base_t::start_node->state, 
                                            node->state); 
            
            return calculateKey(node, h);
        }
                
        static Key_t keyMax()
        {
            return DStarLiteCommon<B>::keyMax();
        }
        
        static bool equal(Key_t const& key1, Key_t const& key2)
        {
            return DStarLiteCommon<B>::equal(key1, key2);
        }
        
        void initialize(State_t start, State_t goal)
        {
            base_t::initialize(start, goal);
                        
            if(base_t::start_node && base_t::goal_node)
            {                
                last_start = start;
                
                base_t::goal_node->visited = true;    
                base_t::goal_node->rhs = 0;
                
                Cost h =  this_final->heuristic(  
                                            base_t::start_node->state, 
                                            base_t::goal_node->state); 

                this_final->pushOpen(base_t::goal_node, 
                        this_final->calculateKey(base_t::goal_node, h));
            }
        }
        
        
        void evaluateNodeRhs(Node_t *node, State_t const& state)
        {
            assert(state != base_t::goal_node->state);
                
            Cost_t min_rhs = Domain_t::costMax();
            bool min_rhs_set = false;
            
            std::vector<Node_t *> &successors_ = base_t::search_space->successors(node);
            
            for(Node_t *neighbournode : successors_)
            {      
                
                if(!neighbournode->visited)
                    continue;
                
                Cost_t rhs;
                
                assert(neighbournode->g > neighbournode->rhs || 
                        Domain_t::equal(neighbournode->g, neighbournode->rhs) || neighbournode->open);
                
                Cost_t succ_cost = this_final->cost(node->state, neighbournode->state);
                
                if(Domain_t::equal(succ_cost, Domain_t::costMax())
                         || Domain_t::equal(neighbournode->g, Domain_t::costMax()))
                {
                    rhs = Domain_t::costMax();
                }
                else
                {
                    rhs = succ_cost + neighbournode->g;
                }
                
                if(!min_rhs_set || min_rhs>rhs)
                {
                    min_rhs_set = true;
                    min_rhs = rhs;
                }
            }
                                                             
            node->rhs = min_rhs;
            node->state = state;
                                    
            if(!node->visited)
            {
                node->visited = true;
            }            
        }
        
        
        void updateVertex(Node_t *node)
        {            
            if(!Domain_t::equal(node->g,node->rhs))
            {
                base_t::open_list->pushOpen(node, recalculateKey(node));
            }
            else if(Domain_t::equal(node->g,node->rhs) && node->open)
            {
                base_t::open_list->removeOpen(node);
            }          
        }
        
        
        void searchStep()
        {                    
            Node_t *node = base_t::open_list->topOpen();            
            base_t::open_list->popOpen();
            
            this_final->searchExpansionStep(node);   
        }
        
        
        void reinitialize(State_t new_start, std::vector<std::pair<StateActionState_t, Cost_t > > const& changed_actions)
        {
            base_t::start_node = base_t::search_space->getNode(new_start);
                        
            if(!changed_actions.empty())
            {
                k_m += base_t::heuristic(last_start,new_start);

                last_start = new_start;

                this_final->reinitializeNodes(changed_actions);

            }    
        }
        
        
        bool isInconsistant(Node_t *node)
        {                               
            if(node==nullptr)
            {
                return false;
            }
            
            Cost_t min_g = Domain_t::costMax();
            Node_t *min_node = nullptr;
            bool min_g_set = false;
            
            std::vector<Node_t *> &successors_ = base_t::search_space->successors(node);  

            for(Node_t *neighbournode : successors_)
            {                                
                if(!neighbournode->visited)
                    continue;
                                
                Cost_t action_cost = this_final->cost(node->state, neighbournode->state);
                
                
                if(Domain_t::equal(action_cost, Domain_t::costMax()))
                    continue;
                
                if(Domain_t::equal(neighbournode->g, Domain_t::costMax()))
                    continue;
                
                Cost_t g = action_cost + neighbournode->g;                              
                
                if(!min_g_set || min_g>g)
                {
                    min_g_set = true;
                    min_g = g;
                    min_node = neighbournode;
                }
            }
            
            return !(node==base_t::goal_node || !min_g_set || Domain_t::equal(node->rhs, min_g) || node->rhs>min_g);                                 
            
        }
        
        Node_t * selectBestNode(Node_t *node)
        {                                    
            Cost_t min_g = Domain_t::costMax();
            Node_t * min_node = nullptr;
            bool min_g_set = false;
            
            std::vector<Node_t *> &successors_ = base_t::search_space->successors(node);  

            for(Node_t *neighbournode : successors_)
            {                      
                if(!neighbournode->visited)
                    continue;
                
                Cost_t action_cost = this_final->cost(node->state, neighbournode->state);
                                
                if(Domain_t::equal(action_cost, Domain_t::costMax()))
                    continue;
                
                if(Domain_t::equal(neighbournode->g, Domain_t::costMax()))
                    continue;
                
                Cost_t g = action_cost + neighbournode->g;                              
                
                if(!min_g_set || min_g>g)
                {
                    min_g_set = true;
                    min_g = g;
                    min_node = neighbournode;
                }
            }
            
            assert(node==base_t::goal_node || !min_g_set || Domain_t::equal(node->rhs, min_g) || node->rhs>min_g);
                                    
            return min_node;
            
        }
        
        State_t actionSelection()
        {                        
            Node_t* min_node = selectBestNode(base_t::start_node);
            
            if(!min_node)
            {
                return base_t::start_node->state;
            }
                        
            return min_node->state;
            
        }
        
        std::vector<State_t> getStatePath(Node_t * node)
        {
            std::vector<State_t> path;
            
            while(node)
            {            
                if(std::find(path.begin(), path.end(), node->state)!=path.end())
                {
                    break;
                }
                
                path.push_back(node->state);     
                
                if(node->state == base_t::goal_node->state)
                {
                    break;
                }
                
                node = selectBestNode(node);                        
            }

            return path;
        }
        
        std::vector<State_t> getStatePath()
        {
            return getStatePath(base_t::start_node);
        }
        
    protected:
        Cost_t k_m;
        State_t last_start;
        typename final_t::Algorithm_t* this_final;
        
    };
};

template<typename B>
class DStarLiteConsistencyCheck : public B
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
        }
        
        virtual ~Algorithm()
        {
            
        }
        
        Key_t minKey()
        {
            Key_t min_key = base_t::keyMax();
            
            auto heap = base_t::open_list->heap();
            
            for(auto heap_pair : heap)
            {                
                if(!heap_pair.second->closed)
                {
                    Key_t k_new = base_t::recalculateKey(static_cast<Node_t*>(heap_pair.second));
                    
                    if(k_new<min_key)
                    {
                        min_key = k_new;
                    }
                }
            }
            
            return min_key;
        }
        
        
        void updateVertex(Node_t *node)
        {                 
            base_t::updateVertex(node);
        }
        
        void searchStep()
        {                    
            Node_t *node = base_t::open_list->topOpen();            
            base_t::open_list->popOpen();
            
            Key_t k_old, k_new;
            
            k_old = node->key;
            k_new = base_t::recalculateKey(node);
            
            Key_t min_key = minKey();                
            assert( Domain_t::equal(min_key.first, k_old.first)
                    || min_key.first > k_old.first);
                        
            assert(!Domain_t::equal(node->g,node->rhs));
            
            this_final->searchExpansionStep(node);   
                                 
            if(!(k_old<k_new))
            {                                
                assert(Domain_t::equal(node->g, node->rhs) || node->g > node->rhs);  
            }
            
        }       
             
        void evaluateNodeRhs(Node_t *node, State_t const& state)
        {            
            base_t::evaluateNodeRhs(node, state);
            
            assert(!base_t::isInconsistant(node));
        }
        
        void consistencyCheck()
        {
            Node_t *top_node = base_t::open_list->topOpen();
            Key_t top_key = minKey();
            
            std::vector<Node_t*> nodes = base_t::search_space->nodes();
            
            for(Node_t* node: nodes)
            {
                if(node && node->visited)
                {
                    Key_t node_key = base_t::recalculateKey(node);
                    
                    if(( (node_key < top_key && !Domain_t::equal(node_key.first, top_key.first))
                            || (Domain_t::equal(node_key.first, top_key.first) && node_key.second < top_key.second)) 
                            && node!=top_node)
                    {
                        assert(node->g > node->rhs  || Domain_t::equal(node->g, node->rhs));
                        base_t::selectBestNode(node);
                    }
                }
            }
        }
                      
        void reinitialize(State_t new_start, std::vector<std::pair<StateActionState_t, Cost_t> > const& changed_actions)
        {
            base_t::reinitialize(new_start, changed_actions);
            consistencyCheck();
        }
        
        bool solutionFound()
        {                                    
            bool found = base_t::solutionFound();

            if(found)
                consistencyCheck();
            
            return found;
        }
               
        std::vector<State_t> getStatePath(Node_t * snode)
        {
            std::vector<State_t> path;
            
            Node_t * node = snode;
            
            while(node)
            {            
                if(std::find(path.begin(), path.end(), node->state)!=path.end())
                {
                    assert(0);
                    break;
                }
                
                path.push_back(node->state);     
                
                if(node->state == base_t::goal_node->state)
                {
                    break;
                }
                
                node = base_t::selectBestNode(node);              
            }

            return path;
        }
        
        std::vector<State_t> getStatePath()
        {
            return getStatePath(base_t::start_node);
        }
        
    protected:
        typename final_t::Algorithm_t* this_final;
        
    };
};

}//namespace heuristic_search

#endif /* DSTARLITECOMMON_H */

