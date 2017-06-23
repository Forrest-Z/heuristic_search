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


#ifndef HEURISTIC_SEARCH_REPEATEDASTAR_H
#define HEURISTIC_SEARCH_REPEATEDASTAR_H

#include <vector>

namespace heuristic_search{


template<typename B>
class RepeatedAStar : public B
{
public:             
    
    typedef typename B::Key_t Key_t;
    
    template<typename final_t>
    class Node : public B::template Node<final_t>
    {
    public:
        typedef typename B::template Node<final_t> base_t;
        
        typedef typename final_t::Domain_t::State State_t;
                
        Node()
        {
            
        }
        
        virtual ~Node()
        {
            
        }
        
        
        void reset()
        {
            base_t::resetNeighbourhoodData();            
            base_t::reset();
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
        
        typedef typename Domain_t::State State_t;
        typedef typename Domain_t::StateActionState StateActionState_t;
        typedef typename Domain_t::Cost Cost_t;
        
        std::string description()
        {
            return "RepeatedA*|" + base_t::description();
        }
        
        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {     
            
        }
        
        virtual ~Algorithm()
        {
            
        }
        
        
        void reinitialize(State_t new_start, std::vector<std::pair<StateActionState_t,Cost_t> > const&)
        {                                                            
            if(base_t::searching_direction == SearchingDirection::Forward)
            {
                base_t::initialize(new_start, base_t::goal_node->state);
            }
            else
            {
                base_t::initialize(new_start, base_t::start_node->state);
            }                    
        }
                        
        State_t actionSelection()
        {
            auto path = 
                static_cast<typename final_t::Algorithm_t*>(this)->getStatePath();
            
            if(path.size()<=1)
            {
                return base_t::start_node->state;
            }
                                    
            return path[1];
        }
        
    };
    
};


}//namespace heuristic_search

#endif /* HEURISTIC_SEARCH_REPEATEDASTAR_H */

