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


#ifndef HEURISTIC_SEARCH_LOGGERS_LOG_H
#define HEURISTIC_SEARCH_LOGGERS_LOG_H

#include <iostream>

namespace heuristic_search{

namespace loggers{


struct Log
{    
    int success;
    double reinitialization_time;
    double search_time;
    double total_time;
    double heap_time;
    double preds_time;
    double succs_time;
    double cost_func_time;
    double path_cost;
    long long int search_steps;
    long long int heap_counter;
    long long int preds_counter;
    long long int succs_counter;
    long long int cost_func_counter;
    
    std::string algorithm_name;
    
    Log()
    {
        success = 0;
        reinitialization_time = 0;
        search_time = 0;
        total_time = 0;
        heap_time = 0;
        preds_time = 0;
        succs_time = 0;
        cost_func_time = 0;
        search_steps = 0;
        heap_counter = 0;
        preds_counter = 0;
        succs_counter = 0;
        cost_func_counter = 0;
        path_cost = 0;
    }
    
    Log& operator+=(Log const& rhs)
    {
        success += rhs.success;
        reinitialization_time += rhs.reinitialization_time;
        search_time += rhs.search_time;
        total_time += rhs.total_time;
        heap_time += rhs.heap_time;
        preds_time += rhs.preds_time;
        succs_time += rhs.succs_time;
        cost_func_time += rhs.cost_func_time;
        search_steps += rhs.search_steps;
        heap_counter += rhs.heap_counter;
        preds_counter += rhs.preds_counter;
        succs_counter += rhs.succs_counter;
        cost_func_counter += rhs.cost_func_counter;
        path_cost += rhs.path_cost;
        
        return *this;
    }
    
    Log& operator/=(int const& divider)
    {
        success /= divider;
        reinitialization_time /= divider;
        search_time /= divider;
        total_time /= divider;
        heap_time /= divider;
        preds_time /= divider;
        succs_time /= divider;
        cost_func_time /= divider;
        search_steps /= divider;
        heap_counter /= divider;
        preds_counter /= divider;
        succs_counter /= divider;
        cost_func_counter /= divider;
        path_cost /= divider;
        
        return *this;
    }
    
    static void writeHeader(std::ostream &stream);
    void writeLog(std::ostream &stream) const;
    
    
};

}//namespace loggers

}//namespace heuristic_search

std::ostream &operator<<(std::ostream & stream, heuristic_search::loggers::Log const &log);

#endif /* HEURISTIC_SEARCH_LOGGERS_LOG_H */

