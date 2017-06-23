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


#ifndef HEURISTIC_SEARCH_DSTAREXTRALITEUNDIRECTED_H
#define HEURISTIC_SEARCH_DSTAREXTRALITEUNDIRECTED_H

#include <vector>


namespace heuristic_search{

template<typename B>
class DStarExtraLiteUndirected_Reinitialize : public B
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
        
        typedef typename Domain_t::State State_t;
        typedef typename Domain_t::StateActionState StateActionState_t;
        typedef typename Domain_t::Cost Cost_t;
        
        std::string description()
        {
            return "D*ExtraLiteUndir.|" + base_t::description();
        }
        
        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {     
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
        }
        
        virtual ~Algorithm()
        {
            
        }
        
        void cutBranch(Node_t *node, std::vector<Node_t*> &seeds)
        {
            base_t::open_list->removeOpen(node);
            node->resetSearchData();
           
            auto &neighbors_ = base_t::neighbors(node);

            for(Node_t *nnode : neighbors_)
            {
                if(nnode->visited)
                {                        
                    if(nnode->parent == node)
                    {
                        cutBranch(nnode, seeds);
                    }
                    else
                    {
                        seeds.push_back(nnode);
                    }
                }
            }    
        }
        
        std::vector<Node_t*> cutBranches(State_t target, 
                std::vector<std::pair<StateActionState_t, Cost_t> > const& updated_actions)
        {
            std::vector<Node_t*> seeds;
            
            Node_t *target_node = base_t::search_space->getNode(target);
                        
            bool reopen_start = false;
                        
            for(auto const& action_pair : updated_actions)
            {
                StateActionState_t const& action = action_pair.first;
                Cost_t const& old_cost = action_pair.second;
                
                State_t const& u_state = action.from;
                State_t const& v_state = action.to;
                
                Node_t *u_node = base_t::search_space->getNode(u_state);
                Node_t *v_node = base_t::search_space->getNode(v_state);
                
                if(!v_node->visited)
                    continue;
                
                if(!u_node->visited)
                {
                    if(!v_node->open)
                    {
                        seeds.push_back(v_node);
                    }
                    
                    continue;
                }
                                                
                Cost_t new_cost = this_final->cost(u_state, v_state);

                if(old_cost < new_cost)
                {
                    Node_t *parent_node = u_node->parent;
                    
                    if(!parent_node || !parent_node->visited)
                        continue;      

                    if(parent_node->state == v_state)
                    {
                        cutBranch(u_node, seeds); 
                    }
                }
                else if(old_cost  > new_cost)
                {
                    Cost_t h = base_t::heuristic(u_state, target);
                    
                    if(v_node->g < Domain_t::costMax() 
                            && h < Domain_t::costMax() 
                            && new_cost < Domain_t::costMax())
                    {                    
                        Cost_t pf = v_node->g + new_cost + h;

                        if(pf < target_node->g)
                        {
                            reopen_start = true;
                        }
                    }

                    seeds.push_back(v_node);
                }
                
            }
            
            if(reopen_start)
            {
                if(target_node->visited)
                {                
                    seeds.push_back(target_node); 
                }
            }
            
            return seeds;
        }
       
        
        void reinitializeNodes(std::vector<Node_t*> &seeds)
        {                         
            for(Node_t *node : seeds)
            {    
                if(node->visited && node->open == false)
                {                
                    Cost_t h = base_t::heuristic(node->state, base_t::goal_node->state);
                    
                    this_final->pushOpen(node, this_final->calculateKey(node, h));
                }
            }
        }
        
        void reinitialize(State_t target, 
                std::vector<std::pair<StateActionState_t, Cost_t> > const& changed_actions)
        {                                    
                        
            if(!changed_actions.empty())
            {                
                std::vector<Node_t*> seeds;
                
                seeds = cutBranches(target, changed_actions);
                
                base_t::reinitialize(target);
                
                if(!seeds.empty())
                {         
                    base_t::increaseKm();
                    reinitializeNodes(seeds); 
                }
            }
            else
            {
                base_t::reinitialize(target);
            }
        }
        
    private:
        typename final_t::Algorithm_t *this_final;
        
    };
    
};



}//namespace heuristic_search

#endif /* HEURISTIC_SEARCH_DSTAREXTRALITEUNDIRECTED_H */

