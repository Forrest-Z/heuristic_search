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


#ifndef MOVINGAI_BENCHMARK_DOMAIN_H
#define MOVINGAI_BENCHMARK_DOMAIN_H

#include <vector>
#include <limits>
#include <cmath>
#include <iostream>
#include <sstream>

namespace movingai_benchmark{


const char UNKNOWN_TERRAIN = '?';
const char PASSABLE_TERRAIN = '.';
const char PASSABLE_TERRAIN_G = 'G';
const char OUT_OF_BOUNDS = '@';
const char OUT_OF_BOUNDS_O = 'O';
const char TREES = 'T';
const char SWAMP = 'S';
const char WATER = 'W';

//. - passable terrain
//G - passable terrain
//@ - out of bounds
//O - out of bounds
//T - trees (unpassable)
//S - swamp (passable from regular terrain)
//W - water (traversable, but not passable from terrain)

class Domain
{
public:

    std::string description()
    {
        std::stringstream desc;

        desc
            << "|H:" << H
            << "|W:"  << W
            << "|Res:" << cell_size_
            << "|Obs:" << observation_range_;

        return desc.str();

    }

    Domain(unsigned char *map, unsigned char *unknown_map, int width, int height,
            int observation_range, int cell_size=1);

    virtual ~Domain();

    typedef int Cost;

    static Cost costMax()
    {
        return std::numeric_limits<Cost>::max();
    }

    static Cost costFactor()
    {
        return 1000;
    }


    struct State
    {
        int x,y;

        explicit State(int x=-1, int y=-1): x(x), y(y)
        {

        }

        bool operator==(State const& rhs) const
        {
            return x==rhs.x && y==rhs.y;
        }

        bool operator!=(State const& rhs) const
        {
            return x!=rhs.x || y!=rhs.y;
        }

        bool operator<(State const& rhs) const
        {
            return x<rhs.x || (x==rhs.x && y<rhs.y);
        }
    };

    struct Action
    {
        Action(Cost cost = costMax()) : cost(cost)
        {

        }

        bool operator==(Action const& rhs) const
        {
            return cost==rhs.cost;
        }

        Cost cost;
    };


    struct StateActionState
    {
        StateActionState()
        {
        }


        StateActionState(State const& from, Action const& action, State const& to) :
            from(from), action(action), to(to)
        {

        }

        State from;
        Action action;
        State to;
    };


    static bool inRange(State const& state, int W, int H)
    {
        if(state.x>=0 && state.x<W && state.y>=0 && state.y<H)
        {
            return true;
        }

        return false;
    }

    bool inRange(State const& state)
    {
        return inRange(state, W, H);
    }

    static int stateId(State const& state, int W, int H)
    {
        if(inRange(state, W, H))
        {
            return state.x+state.y*W;
        }

        return -1;
    }

    int stateId(State const& state)
    {
        return stateId(state, W, H);
    }

    int size()
    {
        return W*H;
    }

    Cost euclideanDistance(State const& s1,State const& s2)
    {
        return std::ceil(costFactor()*cell_size_*std::hypot(s2.x - s1.x, s2.y - s1.y));
    }

    Cost octileDistance(State const& s1, State const& s2)
    {
        int dmin = std::min(std::abs(s2.x-s1.x),std::abs(s2.y-s1.y));
        int dmax = std::max(std::abs(s2.x-s1.x),std::abs(s2.y-s1.y));

        return (costFactor()*cell_size_*(M_SQRT2 * dmin + (dmax-dmin)));
    }

    Cost heuristic(State const& from, State const& to)
    {
        return std::floor(costFactor()*cell_size_*std::sqrt(std::pow(to.x - from.x,2) + std::pow(to.y - from.y,2)));
    }

    static bool equal(Cost const& c1, Cost const& c2)
    {
        return std::abs(c1 - c2) <= 0;
    }

    static Cost domainCost(unsigned char from, unsigned char to);

    Cost cost(State const& from, State const& to, unsigned char *map);

    Cost cost(State const& from, State const& to);


    std::vector<std::pair<StateActionState, Cost> > mapUpdate(State const& current_state);

    std::vector<StateActionState> getActions();

    std::pair<std::vector<StateActionState>, std::vector<StateActionState> >
        getNeighborhood(State state);

    std::vector<StateActionState> getSuccessors(State state);

    std::vector<StateActionState> getPredecessors(State state);


    int cell_size_;
    unsigned char* map_;
    unsigned char* unknown_map_;
    int W,H;
    int observation_range_;

private:
    std::vector<State > affected_states_in_observation_range_;
    std::vector<std::pair<State, State> > affected_actions_in_observation_range_;

    static int cellValueId(unsigned char cell);

    void initializeAffectedActionsInObservationRange();
    void addAffectedActionsToList(State const& current_state,
        std::vector<std::pair<State, State> > &affected_action_list);

    std::vector<std::pair<Domain::StateActionState, Domain::Cost> > scanEnvironment(State const& current_state);
    std::vector<std::pair<Domain::StateActionState, Domain::Cost> > constantRadiusUpdate(State const& current_state);
};

}//namespace movingai_benchmark

std::ostream &operator<<(std::ostream & stream, movingai_benchmark::Domain::State const& state);

#endif /* MOVINGAI_BENCHMARK_DOMAIN_H */
