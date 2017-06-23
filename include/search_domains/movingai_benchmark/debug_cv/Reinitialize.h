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


#ifndef REINITIALIZE_H
#define REINITIALIZE_H

#include "search_domains/movingai_benchmark/debug_cv/opencv.h"

namespace movingai_benchmark{

namespace debug_cv{

template<typename B>
class Reinitialize : public B
{
public:

    template<typename final_t>
    class Algorithm : public B::template Algorithm<final_t>
    {
    public:
        typedef typename B::template Algorithm<final_t> base_t;

        typedef typename final_t::Domain_t::State State_t;
        typedef typename final_t::Domain_t::StateActionState StateActionState_t;

        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {
            cv_scale = 1.0;
            cv_window_delay = 5;
            cv_window_name = "search_debug";
        }

        void reinitialize(State_t new_start, std::vector<StateActionState_t> const& updated_actions)
        {
            base_t::reinitialize(new_start, updated_actions);

            State_t start;

            if(base_t::start_node)
            {
                start = base_t::start_node->state;
            }

            State_t goal;

            if(base_t::goal_node)
            {
                goal = base_t::goal_node->state;
            }

            movingai_benchmark::debug_cv::drawAlgorithm(cv_window_name.c_str(),
                    base_t::domain->map_, base_t::domain->W, base_t::domain->H,
                    start,
                    goal,
                    *this,
                    cv_scale);

            cv::waitKey(cv_window_delay);
        }

        double cv_scale;
        int cv_window_delay;
        std::string cv_window_name;

    private:
        std::vector<StateActionState_t> updated_actions_;

    };

};

}//namespace debug_cv

}//namespace movingai_benchmark

#endif /* REINITIALIZE_H */
