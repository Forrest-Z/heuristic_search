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


#include "search_domains/movingai_benchmark/Domain.h"

#include <cassert>
#include <set>
#include <cstring>
#include <algorithm>

namespace movingai_benchmark{

const int NEIGHBORHOOD_SIZE = 8;

typedef int state_offset_t[2];
const state_offset_t successors_id_offset[NEIGHBORHOOD_SIZE] = {
    {-1,-1},
    {0,-1},
    {1,-1},
    {1,0},
    {1,1},
    {0,1},
    {-1,1},
    {-1,0}
};

const int AFFECTED_SIZE = 24;
typedef state_offset_t affected_offset_t[2];
const affected_offset_t affected_offset[] = {
    {{0,0},{-1,-1}},
    {{0,0},{0,-1}},
    {{0,0},{1,-1}},
    {{0,0},{1,0}},
    {{0,0},{1,1}},
    {{0,0},{0,1}},
    {{0,0},{-1,1}},
    {{0,0},{-1,0}},

    {{-1,-1},{0,0}},
    {{0,-1},{0,0}},
    {{1,-1},{0,0}},
    {{1,0},{0,0}},
    {{1,1},{0,0}},
    {{0,1},{0,0}},
    {{-1,1},{0,0}},
    {{-1,0},{0,0}},

    {{-1,0},{0,1}},
    {{1,0},{0,1}},
    {{1,0},{0,-1}},
    {{-1,0},{0,-1}},
    {{0,1},{-1,0}},
    {{0,1},{1,0}},
    {{0,-1},{1,0}},
    {{0,-1},{-1,0}}
};


//case UNKNOWN_TERRAIN: return 0;
//case PASSABLE_TERRAIN: return 1;
//case PASSABLE_TERRAIN_G: return 2;
//case OUT_OF_BOUNDS: return 3;
//case OUT_OF_BOUNDS_O: return 4;
//case TREES: return 5;
//case SWAMP: return 6;
//case WATER: return 7;

//. - passable terrain
//G - passable terrain
//@ - out of bounds
//O - out of bounds
//T - trees (unpassable)
//S - swamp (passable from regular terrain)
//W - water (traversable, but not passable from terrain)

const int transition_table[8][8] = {
    {1, 1, 1, 0, 0, 0, 1, 1},
    {1, 1, 1, 0, 0, 0, 1, 0},
    {1, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {1, 1, 0, 0, 0, 0, 1, 1},
    {1, 0, 0, 0, 0, 0, 1, 1}
    };

Domain::Domain(unsigned char *map, unsigned char *unknown_map,
        int width, int height, int observation_range, int cell_size)
{
    if(!(width>0 && height>0))
    {
        map_ = nullptr;
        unknown_map_ = nullptr;
        W = width;
        H = height;
        observation_range_ = observation_range;
        cell_size_ = cell_size;
    }
    else
    {
        map_ = new unsigned char[width*height];
        std::memcpy(map_, map, sizeof(unsigned char)*width*height);

        if(unknown_map)
        {
            unknown_map_ = new unsigned char[width*height];
            std::memcpy(unknown_map_, unknown_map, sizeof(unsigned char)*width*height);
        }
        else
        {
            unknown_map_ = nullptr;
        }

        W = width;
        H = height;

        observation_range_ = observation_range;

        cell_size_ = cell_size;

        initializeAffectedActionsInObservationRange();
    }
}


Domain::~Domain()
{
    delete [] map_;
    delete [] unknown_map_;
}

void Domain::initializeAffectedActionsInObservationRange()
{
    if(observation_range_>0)
    {
        for(int i=-observation_range_; i<=observation_range_; ++i)
        {
            int j_range = std::sqrt(std::pow(observation_range_, 2) - std::pow(i,2));
            for(int j=-j_range; j<=j_range; ++j)
            {
                State current_state(i,j);
                affected_states_in_observation_range_.push_back(current_state);

                addAffectedActionsToList(current_state, affected_actions_in_observation_range_);

            }
        }
    }
}

void Domain::addAffectedActionsToList(State const& current_state,
        std::vector<std::pair<State, State> > &affected_action_list)
{
    for(int k=0; k<AFFECTED_SIZE; ++k)
    {
        State from_state = current_state;
        from_state.x += affected_offset[k][0][0];
        from_state.y += affected_offset[k][0][1];

        State to_state = current_state;
        to_state.x += affected_offset[k][1][0];
        to_state.y += affected_offset[k][1][1];

        if(std::find(
                affected_action_list.begin(),
                affected_action_list.end(),
                std::pair<State,State>(from_state, to_state)) ==
                affected_action_list.end())
        {
            affected_action_list.push_back(
                std::pair<State,State>(from_state, to_state));
        }
    }
}

int Domain::cellValueId(unsigned char cell)
{
    switch(cell)
    {
        case UNKNOWN_TERRAIN: return 0;
        case PASSABLE_TERRAIN: return 1;
        case PASSABLE_TERRAIN_G: return 2;
        case OUT_OF_BOUNDS: return 3;
        case OUT_OF_BOUNDS_O: return 4;
        case TREES: return 5;
        case SWAMP: return 6;
        case WATER: return 7;
    }

    return 3;
}

Domain::Cost Domain::domainCost(unsigned char from, unsigned char to)
{
    if(transition_table[cellValueId(from)][cellValueId(to)])
    {
        return static_cast<Cost>(1);
    }

    return costMax();
}

Domain::Cost Domain::cost(State const& from, State const& to, unsigned char *map)
{
    if(!inRange(from) || !inRange(to))
    {
        return costMax();
    }

    unsigned char from_cell = map[stateId(from)];
    unsigned char to_cell = map[stateId(to)];

    Cost _cost = domainCost(from_cell, to_cell);
    double cell_distance = 1;

    //to avoid corner cutting
    if(from.x!=to.x && from.y!=to.y)
    {
        State to_y(from.x, to.y);
        State to_x(to.x, from.y);

        unsigned char to_x_cell = map_[stateId(to_x)];
        unsigned char to_y_cell = map_[stateId(to_y)];

        _cost = std::max(_cost, domainCost(from_cell, to_x_cell));
        _cost = std::max(_cost, domainCost(from_cell, to_y_cell));

        cell_distance = M_SQRT2;
    }


    if(costMax()==_cost)
    {
        return _cost;
    }
    else
    {
        return std::ceil(costFactor()*cell_size_*cell_distance);
    }
}

Domain::Cost Domain::cost(State const& from, State const& to)
{
    return cost(from, to, map_);
}

std::vector<std::pair<Domain::StateActionState, Domain::Cost> > Domain::scanEnvironment(
    State const& current_state)
{

    double step = M_PI*1.0/180.0;

    int cx = current_state.x;
    int cy = current_state.y;

    std::set<State> obstacles;

    std::vector<std::pair<Domain::StateActionState, Domain::Cost> > detected_changes_sas;
    std::vector<std::pair<Domain::State, Domain::State> > potentially_changed_actions;

    for(double angle=0.0; angle<=2.0*M_PI; angle+=step)
    {
        int prev_ex = cx;
        int prev_ey = cy;

        for(int i=1; i<=observation_range_; i++)
        {
            double x;
            double y;

            x = i*std::cos(angle);
            y = std::sin(angle)*i;

            x+=0.5;
            y+=0.5;

            int ex = cx + x;
            int ey = cy + y;

            if(ex<0 || ex>W-1 || ey<0 || ey>H-1 )
                break;

            assert(ex>=0 && ex<W && ey>=0 && ey<H );

            if(map_[ey*W+ex]!=unknown_map_[ey*W+ex])
            {
                State obstacle(ex, ey);
                obstacles.insert(obstacle);

                addAffectedActionsToList(obstacle, potentially_changed_actions);

            }

            if(domainCost(unknown_map_[prev_ey*W+prev_ex], unknown_map_[ey*W+ex])==costMax())
            {
                break;
            }

            prev_ex = ex;
            prev_ey = ey;
        }

    }

    std::vector<StateActionState> previous_sas;
    previous_sas.reserve(potentially_changed_actions.size());

    for(auto affected_action : potentially_changed_actions)
    {
        State from_state;
        from_state.x = affected_action.first.x;
        from_state.y = affected_action.first.y;

        State to_state;
        to_state.x = affected_action.second.x;
        to_state.y = affected_action.second.y;

        Cost known_cost = cost(from_state, to_state);

        Action action(known_cost);
        previous_sas.push_back(StateActionState(from_state, action, to_state));
    }

    for(State obstacle : obstacles)
    {
        int ex = obstacle.x;
        int ey = obstacle.y;

        map_[ey*W+ex]=unknown_map_[ey*W+ex];
    }


    for(auto sas : previous_sas)
    {
        Cost new_cost = cost(sas.from, sas.to);

        if(!equal(sas.action.cost, new_cost))
        {
            Action action(new_cost);
            detected_changes_sas.push_back(
                std::make_pair(StateActionState(sas.from, action, sas.to), sas.action.cost));
        }
    }

    return detected_changes_sas;
}

std::vector<std::pair<Domain::StateActionState, Domain::Cost> > Domain::mapUpdate(State const& current_state)
{
    return scanEnvironment(current_state);
}

std::vector<std::pair<Domain::StateActionState, Domain::Cost> > Domain::constantRadiusUpdate(State const& current_state)
{
    std::vector<std::pair<StateActionState, Cost> > detected_changes_sas;
    std::vector<StateActionState> previous_sas;
    previous_sas.reserve(affected_actions_in_observation_range_.size());

    for(auto affected_action : affected_actions_in_observation_range_)
    {
        State from_state = current_state;
        from_state.x += affected_action.first.x;
        from_state.y += affected_action.first.y;

        State to_state = current_state;
        to_state.x += affected_action.second.x;
        to_state.y += affected_action.second.y;

        if(!inRange(from_state) ||!inRange(to_state) )
        {
            continue;
        }

        Cost known_cost = cost(from_state, to_state);

        Action action(known_cost);
        previous_sas.push_back(StateActionState(from_state, action, to_state));
    }

    for(auto affected_state : affected_states_in_observation_range_)
    {
        State from_state = current_state;
        from_state.x += affected_state.x;
        from_state.y += affected_state.y;

        if(!inRange(from_state))
        {
            continue;
        }

        int from_state_id = stateId(from_state);

        if(map_[from_state_id]!=unknown_map_[from_state_id])
        {
            map_[from_state_id] = unknown_map_[from_state_id];
        }

    }

    for(auto sas : previous_sas)
    {
        Cost new_cost = cost(sas.from, sas.to);

        if(!equal(sas.action.cost, new_cost))
        {
            Action action(new_cost);
            detected_changes_sas.push_back(
                std::make_pair(StateActionState(sas.from, action, sas.to), sas.action.cost));
        }
    }

    return detected_changes_sas;
}


std::vector<Domain::StateActionState> Domain::getActions()
{
    std::vector<StateActionState> actions;

    for(int i=0; i<W; ++i)
    {
        for(int j=0; j<H; ++j)
        {
            State state(i,j);

            std::vector<StateActionState> successors = getSuccessors(state);

            actions.insert(actions.end(), successors.begin(), successors.end());

        }
    }

    return actions;
}

std::pair<std::vector<Domain::StateActionState>, std::vector<Domain::StateActionState> >
Domain::getNeighborhood(State state)
{
    std::vector<StateActionState> successors = getSuccessors(state);
    std::vector<StateActionState> predecessors = getPredecessors(state);

    return std::pair<std::vector<Domain::StateActionState>,
            std::vector<Domain::StateActionState> >(successors, predecessors);
}

std::vector<Domain::StateActionState> Domain::getSuccessors(State state)
{
    std::vector<StateActionState> successors;

    successors.reserve(NEIGHBORHOOD_SIZE);

    for(int i=0; i<NEIGHBORHOOD_SIZE; ++i)
    {
        State neighbor_state = state;
        neighbor_state.x += successors_id_offset[i][0];
        neighbor_state.y += successors_id_offset[i][1];

        if(!inRange(neighbor_state))
        {
            continue;
        }

        successors.push_back(StateActionState(state, Action(), neighbor_state));
    }

    return successors;
}

std::vector<Domain::StateActionState> Domain::getPredecessors(State state)
{
    std::vector<StateActionState> predecessors;

    predecessors.reserve(NEIGHBORHOOD_SIZE);

    for(int i=0; i<NEIGHBORHOOD_SIZE; ++i)
    {
        State neighbor_state = state;
        neighbor_state.x += successors_id_offset[i][0];
        neighbor_state.y += successors_id_offset[i][1];

        if(!inRange(neighbor_state))
        {
            continue;
        }

        predecessors.push_back(StateActionState(neighbor_state, Action(), state));
    }

    return predecessors;
}

}//namespace movingai_benchmark

std::ostream &operator<<(std::ostream & stream, movingai_benchmark::Domain::State const& state)
{
    stream << "(" << state.x << "," << state.y << ")";

    return stream;
}
