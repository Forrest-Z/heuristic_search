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


#ifndef HEURISTIC_SEARCH_INCREMENTALSEARCH_H
#define HEURISTIC_SEARCH_INCREMENTALSEARCH_H

namespace heuristic_search{


template<typename B>
class IncrementalSearch_Initialize : public B
{
public:

    typedef typename B::Key_t Key_t;


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
        typedef typename Domain_t::Cost Cost_t;

        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
            k_m = 0;
        }

        virtual ~Algorithm()
        {

        }

        void increaseKm()
        {
            k_m += base_t::heuristic(last_start, base_t::goal_node->state);
            last_start = base_t::goal_node->state;
        }

        void reinitialize(State_t goal)
        {
            Node_t *new_goal_node = base_t::search_space->getNode(goal);

            if(new_goal_node != base_t::goal_node)
            {
                base_t::goal_node = new_goal_node;

                if(!new_goal_node->visited || new_goal_node->open)
                {
                    increaseKm();
                }
            }

            base_t::finished = false;
            base_t::found = false;
        }

        void initialize(State_t start, State_t goal)
        {
            if(base_t::initialized)
            {
                Node_t *new_start_node = base_t::search_space->getNode(start);
                Node_t *new_goal_node = base_t::search_space->getNode(goal);

                if( base_t::start_node == new_start_node &&
                    base_t::goal_node != new_goal_node)
                {
                    reinitialize(goal);
                }
                else if(base_t::start_node != new_start_node)
                {
                    k_m = 0;
                    last_start = goal;
                    base_t::initialize(start, goal);
                }
            }
            else
            {
                k_m = 0;
                last_start = goal;
                base_t::initialize(start, goal);
            }

        }

        Key_t calculateKey(Node_t *node, Cost_t h)
        {
            Key_t base_key = base_t::calculateKey(node, h);
            if(Domain_t::equal(base_key, Domain_t::costMax()))
            {
                return base_key;
            }

            return base_key + k_m;
        }

        State_t last_start;
        Cost_t k_m;
    private:
        typename final_t::Algorithm_t *this_final;
    };

};


template<typename B>
class IncrementalSearch_SearchStep : public B
{
public:


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
        typedef typename Domain_t::Cost Cost_t;

        typedef typename final_t::Key_t Key_t;

        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
        }

        virtual ~Algorithm()
        {

        }

        void searchStep()
        {
            Node_t *node = base_t::open_list->topOpen();
            Key_t prev_key = base_t::open_list->topKey();

            Key_t new_key = this_final->calculateKey(
                    node,
                    base_t::heuristic(node->state, base_t::goal_node->state));

            if(new_key > prev_key && !final_t::equal(new_key, prev_key))
            {
                this_final->pushOpen(node, new_key);
            }
            else
            {
                base_t::searchStep();
            }
        }

    private:
        typename final_t::Algorithm_t *this_final;
    };

};

}//namespace heuristic_search


#endif /* HEURISTIC_SEARCH_INCREMENTALSEARCH_H */
