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


#ifndef MPGAALOGGER_H
#define MPGAALOGGER_H

#include "heuristic_search/loggers/DStarLogger.h"

namespace heuristic_search{

namespace loggers{

template<typename B>
class MPGAALogger : public DStarLogger<B>
{
public:
    
    template<typename final_t> 
    class Algorithm : public DStarLogger<B>::template Algorithm<final_t>
    {
    public:
        typedef typename DStarLogger<B>::template Algorithm<final_t> base_t;
        
        typedef typename final_t::Domain_t Domain_t;
        typedef typename final_t::Domain_t::State State;
        typedef typename final_t::Domain_t::Cost Cost_t;
        
        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {
            base_t::h_open_list->log = &(base_t::log);
        }
        
        bool reestablishConsistencyStep()
        {
            ++(base_t::log.search_steps);
            return base_t::reestablishConsistencyStep();
        }
    };
           
    
    template<typename final_t>
    class HOpenList : public DStarLogger<B>::template HOpenList<final_t>
    {
    public:
        typedef typename DStarLogger<B>::template HOpenList<final_t> base_t;
        
        typedef typename final_t::Node_t Node_t;
        typedef typename final_t::Key_t Key_t;
        typedef typename final_t::Domain_t Domain_t;
        
        typedef std::pair<Key_t, Node_t*> heap_pair_t;
        typedef std::vector<heap_pair_t> heap_container_t;
        
        
        template<typename ...Args>
        HOpenList(Args&&... args) : base_t(std::forward<Args>(args)...)
        {          
            log = nullptr;
        }
        
        virtual ~HOpenList()
        {
            
        }
                
        void popOpen()
        {
            log->heap_counter++;             
            base_t::popOpen();            
        }
        
        void pushOpen(Node_t *v, Key_t key)
        {            
            log->heap_counter++;             
            base_t::pushOpen(v, key); 
        }
        
        void removeOpen(Node_t *v)
        {    
            log->heap_counter++;
            base_t::removeOpen(v); 
        }
        
        Log *log;
            
    };
};

}//namespace loggers

}//namespace heuristic_search

#endif /* MPGAALOGGER_H */

