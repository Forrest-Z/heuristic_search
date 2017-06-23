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


#include "heuristic_search/loggers/Log.h"

#include <iomanip>
#include <cmath>

namespace heuristic_search{

namespace loggers{

void Log::writeHeader(std::ostream &stream)
{
    int width = 11;
    
    stream           
        << std::setw(width) << std::right << "success"
        << std::setw(width) << std::setprecision(3) << std::right << "reinit_t"
        << std::setw(width) << std::setprecision(3) << std::right << "search_t"
        << std::setw(width) << std::setprecision(3) << std::right << "total_t"
        << std::setw(width) << std::right << "steps_ct"
        << std::setw(width) << std::right << "heap_ct"
        << std::setw(width) << std::right << "preds_ct"
        << std::setw(width) << std::right << "succs_ct"
        << std::setw(width) << std::right << "cost_ct"
        << std::setw(width) << std::right << "p_cost"
        << " " << std::setw(30) << std::left << "algorithm_description";
}

void Log::writeLog(std::ostream &stream) const
{
    int width = 11;
    
    stream  << std::fixed 
            << std::setprecision(2)             
        << std::setw(width) << std::right << success 
        << std::setw(width) << std::right << std::round(1e5 * reinitialization_time)/1e2 
        << std::setw(width) << std::right << std::round(1e5 * search_time )/1e2  
        << std::setw(width) << std::right << std::round(1e5 * total_time)/1e2  
        << std::setw(width) << std::right << search_steps 
        << std::setw(width) << std::right << heap_counter
        << std::setw(width) << std::right << preds_counter 
        << std::setw(width) << std::right << succs_counter 
        << std::setw(width) << std::right << cost_func_counter 
        << std::setprecision(3)    
        << std::setw(width) << std::right << std::round(1e3 *path_cost )/1e3  
        << " " << std::setw(30) << std::left << algorithm_name;
}

}//namespace loggers

}//namespace heuristic_search

std::ostream &operator<<(std::ostream & stream, heuristic_search::loggers::Log const &log)
{
    log.writeLog(stream);

    return stream;
}
