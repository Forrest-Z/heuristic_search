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


#ifndef HEURISTIC_SEARCH_LOGGERS_DSTARLOGGER_H
#define HEURISTIC_SEARCH_LOGGERS_DSTARLOGGER_H

#include "heuristic_search/loggers/Logger.h"

namespace heuristic_search{

namespace loggers{

template<typename B>
class DStarLogger : public Logger<B>
{
public:
    
    template<typename final_t> 
    class Algorithm : public Logger<B>::template Algorithm<final_t>
    {
    public:
        typedef typename Logger<B>::template Algorithm<final_t> base_t;
        
        typedef typename final_t::Domain_t Domain_t;
        typedef typename final_t::Domain_t::State State_t;
        typedef typename final_t::Domain_t::Cost Cost_t;
        typedef typename final_t::Domain_t::StateActionState StateActionState_t;
        
        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {
            
        }
    
        void main(State_t start, State_t goal)
        { 
            base_t::main(start, goal);
            
            if(!(base_t::current_state==goal))
            {
                base_t::log.success = 0;
            }
            else
            {
                base_t::log.success = 1;
            }
            
            base_t::log.total_time = base_t::log.search_time + base_t::log.reinitialization_time;
            base_t::log.path_cost = base_t::travelled_path_cost/double(Domain_t::costFactor());
        }
        
        void reinitialize(State_t new_start, std::vector<std::pair<StateActionState_t, Cost_t> > const& updated_actions)
        {            
            auto t1 = std::chrono::high_resolution_clock::now();
            
            base_t::reinitialize(new_start, updated_actions);
            
            base_t::log.reinitialization_time += std::chrono::duration_cast< std::chrono::nanoseconds>(
                        std::chrono::system_clock::now() - t1).count()/1.0e9;
            
        }

    
    };
        
};

}//namespace loggers

}//namespace heuristic_search

#endif /* HEURISTIC_SEARCH_LOGGERS_DSTARLOGGER_H */

