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


#ifndef HEURISTIC_SEARCH_LOGGERS_LOGGER_H
#define HEURISTIC_SEARCH_LOGGERS_LOGGER_H

#include <chrono>
#include <vector>

#include "heuristic_search/loggers/Log.h"

namespace heuristic_search{

namespace loggers{

template<typename B>
class Logger : public B
{
public:
    
    template<typename final_t> 
    class Algorithm : public B::template Algorithm<final_t>
    {
    public:
        typedef typename B::template Algorithm<final_t> base_t;
        
        typedef typename final_t::Domain_t Domain_t;
        typedef typename final_t::Domain_t::State State;
        typedef typename final_t::Domain_t::Cost Cost_t;
        
        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {
            base_t::search_space->log = &log;
            base_t::open_list->log = &log;
            log.algorithm_name = base_t::description();
            
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
        }
    
        bool search()
        {
            auto t1 = std::chrono::high_resolution_clock::now();
            
            bool result = base_t::search();
            
            log.search_time += std::chrono::duration_cast< std::chrono::nanoseconds>(
                        std::chrono::system_clock::now() - t1).count()/1.0e9;
            
            if(result)
            {
                log.success++;
            }
            
            return result;
        }
        
        bool search(State start, State goal)
        {
            base_t::search(start, goal);
            log.total_time = log.search_time + log.reinitialization_time;
            
            if(log.success)
            {                
                auto path = base_t::getStatePath();

                Cost_t cost = 0;

                for(int i=1; i<int(path.size()); ++i)
                {
                    cost += base_t::domain->cost(path[i-1], path[i]);
                }
                
                log.path_cost = cost/double(Domain_t::costFactor());
            }
            
            return log.success;
        }
        
        void initialize(State start, State goal)
        {
            auto t1 = std::chrono::high_resolution_clock::now();
            base_t::initialize(start, goal);
            log.reinitialization_time += std::chrono::duration_cast< std::chrono::nanoseconds>(
                       std::chrono::system_clock::now() - t1).count()/1.0e9;
        }
                
        Cost_t cost(State from, State to)
        {
            ++(log.cost_func_counter);            
            return base_t::cost(from, to);
        }
        
        void searchStep()
        {
            ++log.search_steps;
            base_t::searchStep();
        }
        
        Log result()
        {
            return log;
        }

        Log log;
    private:
        typename final_t::Algorithm_t *this_final;
    
    };
    
    template<typename final_t>
    class SearchSpace : public B::template SearchSpace<final_t>
    {
    public:
        typedef typename B::template SearchSpace<final_t> base_t;
        
        typedef typename final_t::Node_t Node_t;

        typedef typename final_t::Domain_t Domain_t;
        typedef typename final_t::Domain_t::State State;
        typedef typename final_t::Domain_t::Action Action;
        typedef typename final_t::Domain_t::StateActionState StateActionState;
        
        template<typename ...Args>
        SearchSpace(Args&&... args) : base_t(std::forward<Args>(args)...)
        {          
            log = nullptr;
        }
        
        virtual ~SearchSpace()
        {
            
        }

                
        std::vector<Node_t*> &predecessors(Node_t *node)
        {
            ++log->preds_counter;
            return base_t::predecessors(node);
        }

        std::vector<Node_t*> &successors(Node_t *node)
        {
            ++log->succs_counter;
            return base_t::successors(node);
        }
        
        Log *log;
    };
    
    
    template<typename final_t>
    class OpenList : public B::template OpenList<final_t>
    {
    public:
        typedef typename B::template OpenList<final_t> base_t;
        
        typedef typename HeuristicSearch<typename final_t::HeuristicSearch_base_t>::template Node<final_t> Node_t;
        typedef typename final_t::Key_t Key_t;
        typedef typename final_t::Domain_t Domain_t;
        
        typedef std::pair<Key_t, Node_t*> heap_pair_t;
        typedef std::vector<heap_pair_t> heap_container_t;
        
        
        template<typename ...Args>
        OpenList(Args&&... args) : base_t(std::forward<Args>(args)...)
        {          
            log = nullptr;
        }
        
        virtual ~OpenList()
        {
            
        }
                                
        void popOpen()
        {
            log->heap_counter++;
            base_t::popOpen();
        }
        
        void pushOpen(Node_t *v, Key_t key)
        {            
            if(!(v->open && !v->closed && v->key == key))
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

#endif /* HEURISTIC_SEARCH_LOGGERS_LOGGER_H */

