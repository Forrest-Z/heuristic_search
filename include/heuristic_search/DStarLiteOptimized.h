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


#ifndef DSTARLITEOPTIMIZED_H
#define DSTARLITEOPTIMIZED_H

#include "heuristic_search/DStarLiteCommon.h"

namespace heuristic_search{

template<typename B>
class DStarLiteOptimized : public DStarLiteCommon<B>
{
public:

    typedef typename DStarLiteCommon<B>::Domain_t::Cost Cost;
    typedef typename DStarLiteCommon<B>::Key_t Key_y;


    template<typename final_t>
    class Algorithm : public DStarLiteCommon<B>::template Algorithm<final_t>
    {
    public:
        typedef typename DStarLiteCommon<B>::template Algorithm<final_t> base_t;

        typedef typename final_t::Domain_t Domain_t;
        typedef typename final_t::OpenList_t OpenList_t;
        typedef typename final_t::Node_t Node_t;
        typedef typename final_t::SearchSpace_t SearchSpace_t;
        typedef typename final_t::Key_t Key_t;

        typedef typename Domain_t::State State_t;
        typedef typename Domain_t::StateActionState StateActionState_t;
        typedef typename Domain_t::Cost Cost_t;

        std::string description()
        {
            return "D*LiteOpt.|" + base_t::description();
        }

        template<typename ...Args>
        Algorithm(Args&&... args) : base_t(std::forward<Args>(args)...)
        {
            this_final = static_cast<typename final_t::Algorithm_t*>(this);
        }

        virtual ~Algorithm()
        {

        }


        bool solutionFound()
        {
            Node_t *top_node = base_t::open_list->topOpen();

            Key_t start_key = base_t::recalculateKey(base_t::start_node);

            bool found = (top_node->key > start_key || base_t::equal(top_node->key, start_key))
                    && (base_t::start_node->rhs < base_t::start_node->g
                        || Domain_t::equal(base_t::start_node->rhs,  base_t::start_node->g));

            return found;
        }


        void searchExpansionStep(Node_t *node)
        {

            Cost_t g;
            Key_t k_old, k_new;

            k_old = node->key;
            k_new = this_final->recalculateKey(node);

            if(k_old<k_new)
            {
                base_t::open_list->pushOpen(node, k_new);
            }
            else if(node->g > node->rhs)
            {
                node->g = node->rhs;
                base_t::open_list->removeOpen(node);

                std::vector<Node_t *> &predecessors_ = base_t::search_space->predecessors(node);

                for(Node_t *neighbournode  : predecessors_)
                {
                    State_t nstate = neighbournode->state;

                    assert(neighbournode->g > neighbournode->rhs ||
                        Domain_t::equal(neighbournode->g, neighbournode->rhs) || neighbournode->open);

                    Cost_t pred_cost = this_final->cost(neighbournode->state, node->state);

                    if(nstate!=base_t::goal_node->state)
                    {
                        if(Domain_t::equal(pred_cost, Domain_t::costMax())
                         || Domain_t::equal(node->g, Domain_t::costMax()))
                        {
                            g = Domain_t::costMax();
                        }
                        else
                        {
                            g = node->g + pred_cost;
                        }

                        neighbournode->rhs = std::min(neighbournode->rhs,g);

                        if(!neighbournode->visited)
                        {
                            neighbournode->visited = true;
                        }
                    }

                    this_final->updateVertex(neighbournode);
                }

            }
            else
            {
                Cost_t g_old = node->g;
                node->g = Domain_t::costMax();

                std::vector<Node_t*> &predecessors_ = base_t::search_space->predecessors(node);

                for(Node_t *neighbournode : predecessors_)
                {
                    assert(neighbournode->g > neighbournode->rhs ||
                        Domain_t::equal(neighbournode->g, neighbournode->rhs) || neighbournode->open);

                    State_t nstate = neighbournode->state;

                    Cost_t pred_cost = this_final->cost(neighbournode->state, node->state);

                    if(Domain_t::equal(neighbournode->rhs, g_old + pred_cost)
                            && nstate!=base_t::goal_node->state)
                    {
                        this_final->evaluateNodeRhs(neighbournode,nstate);
                    }

                    this_final->updateVertex(neighbournode);
                }

                if(Domain_t::equal(node->rhs, g_old ) && node->state!=base_t::goal_node->state)
                {
                    this_final->evaluateNodeRhs(node,node->state);
                }

                this_final->updateVertex(node);
            }
        }

        void reinitializeNodes(std::vector<std::pair<StateActionState_t, Cost_t> > const& updated_actions)
        {

            for(auto const& updated_action_pair : updated_actions)
            {
                StateActionState_t const& updated_action = updated_action_pair.first;
                Cost_t const& old_cost = updated_action_pair.second;

                State_t const& from_state = updated_action.from;
                Node_t *from_node = base_t::search_space->getNode(from_state);

                State_t to_state = updated_action.to;
                Node_t * to_node = base_t::search_space->getNode(to_state);

                Cost_t new_cost = this_final->cost(from_state, to_state);


                if(old_cost > new_cost)
                {
                    if(to_node->state!=base_t::goal_node->state)
                    {
                        if(!Domain_t::equal(to_node->g, Domain_t::costMax())
                                && !Domain_t::equal(new_cost, Domain_t::costMax()))
                        {
                            Cost_t g = to_node->g + new_cost;
                            from_node->rhs = std::min(from_node->rhs,g);

                            if(!from_node->visited )
                            {
                                from_node->visited = true;
                            }
                        }
                    }
                }
                else if(old_cost < new_cost)
                {
                    if(from_node->state!=base_t::goal_node->state)
                    {
                        Cost_t to_cost = Domain_t::costMax();

                        if(!Domain_t::equal(Domain_t::costMax(), old_cost)
                                && !Domain_t::equal(Domain_t::costMax(), to_node->g))
                        {
                            to_cost = old_cost + to_node->g;
                        }

                        if(Domain_t::equal(from_node->rhs, to_cost))
                        {
                            this_final->evaluateNodeRhs(from_node, from_state);
                        }
                    }
                }

                this_final->updateVertex(from_node);
            }


        }

    protected:
        typename final_t::Algorithm_t* this_final;

    };
};

}//namespace heuristic_search


#endif /* DSTARLITEOPTIMIZED_H */
