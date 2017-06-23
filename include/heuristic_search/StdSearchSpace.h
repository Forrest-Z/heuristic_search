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


#ifndef STDSEARCHSPACE_H
#define STDSEARCHSPACE_H

#include <vector>
#include <forward_list>
#include <limits>
#include <cassert>

namespace heuristic_search{

const int search_space_buffer_size = 10000;

template<typename B>
class StdSearchSpace : public B
{
public:    
    
    template<typename final_t>
    class Node : public B::template Node<final_t>
    {
    public:
        typedef typename final_t::Domain_t::State State_t;
        typedef typename final_t::Node_t Node_t;     
                
        Node()
        {
            expanded_preds = false;
            expanded_succs = false;
            initialized = false;            
        }
        
        virtual ~Node()
        {
            predecessors.clear();
            successors.clear();
        }
        
        void resetNeighbourhoodData()
        {
            expanded_preds = false;
            expanded_succs = false;
            initialized = false;
            
            predecessors.clear();
            successors.clear();
            
        }
        
        State_t state;
        
        bool expanded_preds;
        bool expanded_succs;
        bool initialized;
        
        std::vector<Node_t*> predecessors;
        std::vector<Node_t*> successors;
        
    };
    
    
    template<typename final_t>
    class SearchSpace : public B::template SearchSpace<final_t>
    {
    public:
        
        typedef typename final_t::Node_t Node_t;

        typedef typename final_t::Domain_t Domain_t;
        typedef typename final_t::Domain_t::State State;
        typedef typename final_t::Domain_t::Cost Cost_t;
        typedef typename final_t::Domain_t::StateActionState StateActionState;
        
        SearchSpace(Domain_t &domain, bool undirected = false) : 
            domain_(&domain), undirected_(undirected)
        {            
            nodes_.push_back(new Node_t[search_space_buffer_size]);
        }
        
        virtual ~SearchSpace()
        {
            for(Node_t *nodes : nodes_)
                delete [] nodes;
            nodes_.clear();
        }

        Node_t *getNode(State const& state)
        {
            int id = domain_->stateId(state);
            
            Node_t *node = nullptr;

            if(id<0)
            {
                return nullptr;
            }
            
            int nodes_id = id/search_space_buffer_size;
            if(nodes_id >= nodes_.size())
            {
                nodes_.resize(nodes_id+1, nullptr);
            }
            
            assert(nodes_id < int(nodes_.size()));
            
            if(nodes_[nodes_id]==nullptr)
            {   
                nodes_[nodes_id] = new Node_t[search_space_buffer_size];
            }

            node = &nodes_[nodes_id][id%search_space_buffer_size];
                        
            if(!node->initialized)
            {
                node->state = state;
                node->initialized = true;
            }

            return node;
        }
        
                
        void initialize()
        {
            int size = domain_->size();

            if(size > 0 && size < std::numeric_limits<int>::max())
            {
                int number_of_buffers = size/search_space_buffer_size + 1;
                
                nodes_.resize(number_of_buffers, nullptr);                
                
                for(int i=0; i < number_of_buffers; ++i)
                {
                    if(nodes_[i]==nullptr)
                    {
                        nodes_[i] = new Node_t[search_space_buffer_size];
                    }
                }
                
                std::vector<StateActionState> actions = domain_->getActions();

                setActions(actions);
                
                for(Node_t *nodes : nodes_)
                {
                    if(nodes)
                    {
                        for(int i=0; i<search_space_buffer_size; ++i)
                        {                        
                            assert(nodes[i].expanded_preds == false);
                            assert(nodes[i].expanded_succs == false);
                            
                            nodes[i].expanded_preds = true;
                            nodes[i].expanded_succs = true;
                        }
                    }
                }
            }
            
        }
              

        std::vector<Node_t*> &predecessors(Node_t *node)
        {
            if(undirected_)
            {
                return successors(node);
            }
            
            if(!node->expanded_preds)
            {
                assert(node->predecessors.empty());
                expandPredecessors(node);
            }

            return node->predecessors;
        }

        std::vector<Node_t*> &successors(Node_t *node)
        {
            if(!node->expanded_succs)
            {
                assert(node->successors.empty());
                expandSuccessors(node);
            }

            return node->successors;
        }
        
        void resetSearchData()
        {                        
            for(Node_t *nodes : nodes_)
            {
                if(nodes)
                {
                    for(int i=0; i<search_space_buffer_size; ++i)
                    {                        
                        nodes[i].resetSearchData();
                    }
                }
            }     
        }
        
        void resetNeighbourhoodData()
        {                        
            for(Node_t *nodes : nodes_)
            {
                if(nodes)
                {
                    for(int i=0; i<search_space_buffer_size; ++i)
                    {                        
                        nodes[i].resetNeighbourhoodData();
                    }
                }
            }     
        }

        std::vector<Node_t*> nodes() const
        {
            std::vector<Node_t*> _nodes;
            
            for(Node_t *node_array : nodes_)
            {
                if(node_array)
                {
                    for(int i=0; i<search_space_buffer_size; ++i)
                    {
                        Node_t *node = &node_array[i];
                        
                        if(node->initialized)
                        {
                            _nodes.push_back(node);
                        }
                    }
                }
                
            }
            
            return _nodes;
        }
        
        std::vector<Node_t*> const& nodeBuffers() const
        {
            return nodes_;
        }
        
        void setActions(std::vector<StateActionState> const& actions)
        {
            for(auto action : actions)
            {
                setAction(action);
            }
        }
               
        int bufferSize()
        {
            return search_space_buffer_size;
        }
                
    protected:    
        Domain_t *domain_;
        std::vector<Node_t*> nodes_;
        bool undirected_;
        
        void expandPredecessors(Node_t *node)
        {
            std::vector<StateActionState> preds = domain_->getPredecessors(node->state);
            
            setPredecessors(node, preds);
            
            node->expanded_preds = true;
        }
        
        void expandSuccessors(Node_t *node)
        {
            std::vector<StateActionState> succs = domain_->getSuccessors(node->state);
            
            setSuccessors(node, succs);
            
            node->expanded_succs = true;
        }
        
        void setAction(StateActionState const& sas)
        {
            Node_t *from_node = getNode(sas.from);            
            Node_t *to_node = getNode(sas.to);
                        
            setSuccessor(from_node, to_node);
            setPredecessor(to_node, from_node);
        }
        
        
        void setPredecessor(Node_t *node, Node_t *pred)
        {                            
            node->predecessors.push_back(pred);
        }
        
        void setSuccessor(Node_t *node, Node_t *succ)
        {                        
            node->successors.push_back(succ);
        }

        void setPredecessors(Node_t *node, std::vector<StateActionState> const& preds)
        {            
            for(auto pred : preds)
            {
                Node_t *neighbor_node = getNode(pred.from);
                
                setPredecessor(node, neighbor_node);
            }
        }

        void setSuccessors(Node_t *node, std::vector<StateActionState> const& succs)
        {            
            for(auto succ : succs)
            {
                Node_t *neighbor_node = getNode(succ.to);
                
                setSuccessor(node, neighbor_node);
            }
        }
        
    };
    
};

}//namespace heuristic_search

#endif /* STDSEARCHSPACE_H */

