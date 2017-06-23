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


#ifndef DSTARLITE_H
#define DSTARLITE_H

#include "heuristic_search/DStarLiteCommon.h"

namespace heuristic_search{

template<typename B>
class DStarLite : public DStarLiteCommon<B>
{
public:             
    
    typedef typename DStarLiteCommon<B>::Domain_t::Cost Cost;
    typedef typename DStarLiteCommon<B>::Key_t Key_y;    
           
    
    template<typename final_t> 
    class Algorithm : public DStarLiteCommon<B>::template Algorithm<final_t>
    {
    public:
        typedef typename DStarLiteCommon<B>::template Algorithm<final_t> base_t;
        
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
            return "D*Lite|" + base_t::description();
        }
        
        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {     
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
        }
        
        virtual ~Algorithm()
        {
            
        }
        
        
        bool solutionFound()
        {            
            Node_t *top_node = base_t::open_list->topOpen();
            
            Key_t start_key = base_t::recalculateKey(base_t::start_node);
            
            bool found = (top_node->key > start_key || base_t::equal(top_node->key, start_key))
                    && Domain_t::equal(base_t::start_node->rhs,  base_t::start_node->g);
            
            return found;
        }
        
                       
        void searchExpansionStep(Node_t *node)
        {                       
            Key_t k_old, k_new;
            
            k_old = node->key;
            k_new = base_t::recalculateKey(node);
            
            if(k_old<k_new)
            {
                base_t::open_list->pushOpen(node, k_new);                
            }
            else if(node->g > node->rhs)
            {
                node->g = node->rhs;
                
                std::vector<Node_t*> &predecessors_ = base_t::search_space->predecessors(node);

                for(Node_t *neighbournode : predecessors_)
                {      
                    State_t nstate = neighbournode->state;
                    
                    assert(neighbournode->g > neighbournode->rhs || 
                        Domain_t::equal(neighbournode->g, neighbournode->rhs) || neighbournode->open);
                    
                    if(nstate!=base_t::goal_node->state)
                    {
                        this_final->evaluateNodeRhs(neighbournode, nstate);
                    }
                                        
                    if(neighbournode->open)
                    {
                        base_t::open_list->removeOpen(neighbournode);
                    }
                    
                    this_final->updateVertex(neighbournode);
                }
                                
            }
            else
            {
                node->g = Domain_t::costMax();
                
                std::vector<Node_t*> &predecessors_ = base_t::search_space->predecessors(node);

                for(Node_t *neighbournode : predecessors_)
                {      
                    assert(neighbournode->g > neighbournode->rhs || 
                        Domain_t::equal(neighbournode->g, neighbournode->rhs) || neighbournode->open);
                    
                    State_t nstate = neighbournode->state;                    
                    
                    if(nstate!=base_t::goal_node->state)
                    {
                        this_final->evaluateNodeRhs(neighbournode,nstate);
                    }
                    
                    if(neighbournode->open)
                    {
                        base_t::open_list->removeOpen(neighbournode);
                    }
                    this_final->updateVertex(neighbournode);
                }
                
                if(node->state!=base_t::goal_node->state)
                {
                    this_final->evaluateNodeRhs(node,node->state);    
                }
                
                if(node->open)
                {
                    base_t::open_list->removeOpen(node);
                }            
                this_final->updateVertex(node);
                
            }            
        }
        
        void reinitializeNodes(std::vector<std::pair<StateActionState_t, Cost_t> > const& updated_actions)
        {
                                    
            for(auto const& updated_action_pair : updated_actions)
            {
                StateActionState_t const& updated_action = updated_action_pair.first;
                
                State_t const& from_state = updated_action.from;                                
                Node_t *from_node = base_t::search_space->getNode(from_state);                                     
                                                    
                if(from_node->visited )
                {
                    if(from_state != base_t::goal_node->state)
                    {
                        this_final->evaluateNodeRhs(from_node, from_state);   
                        
                        if(from_node->open)
                        {
                            base_t::open_list->removeOpen(from_node);
                        }

                        this_final->updateVertex(from_node);    
                    }
                }
            }                            
            
        }
        
    protected:
        typename final_t::Algorithm_t* this_final;
        
    };
};

}//namespace heuristic_search

#endif /* DSTARLITE_H */

