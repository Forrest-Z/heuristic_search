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


#ifndef HEURISTIC_SEARCH_DSTARMAIN_H
#define HEURISTIC_SEARCH_DSTARMAIN_H

#include <vector>
#include <tuple>

namespace heuristic_search{

template<typename B>
class DStarMain : public B
{
public:

    template<typename final_t>
    class Algorithm : public B::template Algorithm<final_t>
    {
    public:
        typedef typename B::template Algorithm<final_t> base_t;

        typedef typename final_t::Domain_t::State State;
        typedef typename final_t::Domain_t::StateActionState StateActionState;
        typedef typename final_t::Domain_t::State State_t;
        typedef typename final_t::Domain_t::Cost Cost;
        typedef typename final_t::Domain_t Domain_t;


        template<typename ...Args>
        Algorithm(Args&&... args) : B::template Algorithm<final_t>(std::forward<Args>(args)...)
        {
            travelled_path_cost = 0;

            this_final = static_cast<typename final_t::Algorithm_t*>(this);
        }

        void update(State_t const& current_state)
        {
            std::vector<std::pair<StateActionState, Cost> > updated_actions =
                    base_t::domain->mapUpdate(current_state);

            this_final->reinitialize(current_state, updated_actions);
        }

        void mainInitialize(State start, State goal)
        {
            this_final->initialize(start, goal);

            auto changed_states = base_t::domain->mapUpdate(start);

            if(!changed_states.empty())
            {
                this_final->reinitialize(start, changed_states);
            }
        }

        void main(State start, State goal)
        {
            this_final->mainInitialize(start, goal);

            current_state = start;

//#ifndef NDEBUG
            int steps = 0;
//#endif
            while(current_state != goal)
            {
                this_final->search();

                if(this_final->finished && !this_final->found)
                    break;

                State_t next_state = this_final->actionSelection();

                assert(next_state != current_state);

                travelled_path_cost += this_final->cost(current_state, next_state);
                current_state = next_state;

                this_final->update(current_state);

                steps++;
#ifndef NDEBUG
                assert(steps<5000);
#endif
                if(steps>=980000)
                {
                    break;
                }
            }
        }

        Cost travelled_path_cost;
        State current_state;

    private:
        typename final_t::Algorithm_t *this_final;

    };

};

}//namespace heuristic_search
#endif /* HEURISTIC_SEARCH_DSTARMAIN_H */
