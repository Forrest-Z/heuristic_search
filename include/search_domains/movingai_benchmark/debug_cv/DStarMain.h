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


#ifndef MOVINGAI_BENCHMARK_DEBUG_CV_DSTARMAIN_H
#define MOVINGAI_BENCHMARK_DEBUG_CV_DSTARMAIN_H

#include "search_domains/movingai_benchmark/debug_cv/debug_cv.h"

namespace movingai_benchmark{

namespace debug_cv{

template<typename B>
class DStarMain : public B
{
public:
    
    template<typename final_t> 
    class Algorithm : public B::template Algorithm<final_t>
    {
    public:
        typedef typename B::template Algorithm<final_t> base_t;
        
        typedef typename final_t::Node_t Node_t;
        typedef typename final_t::Domain_t::State State;
        
        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {
           
        }
    
        
        void main(State start, State goal)
        {                        
            std::vector<std::string> text;
            
            if(!base_t::DebugCVSearch_t::cv_hide_text)
            {    
                text.push_back(base_t::DebugCVSearch_t::cv_window_name);
                text.push_back("press ESC to quit");
                text.push_back("or any other key to continue...");
            }
            
            movingai_benchmark::debug_cv::drawAlgorithm(
                    base_t::DebugCVSearch_t::cv_window_name.c_str(), 
                    base_t::domain->unknown_map_, base_t::domain->W, base_t::domain->H, 
                    start, 
                    goal, 
                    *this, 
                    base_t::DebugCVSearch_t::cv_scale,
                    text);
            
            int key = cv::waitKey(0);
            
            if(key == 27)
            {
                base_t::found = false;
                return;                
            }
            
            base_t::main(start, goal);
        }
                
    };
    
};

}//namespace debug_cv

}//namespace movingai_benchmark

#endif /* MOVINGAI_BENCHMARK_DEBUG_CV_DSTARMAIN_H */

