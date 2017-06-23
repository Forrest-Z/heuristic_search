/* heuristic_search library
 *
 * Copyright (c) 2016, 
 * Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>, 
 * Warsaw University of Technology.
 * All rights reserved.
 *  
 */

#ifndef TEST_TESTDOMAIN_H
#define TEST_TESTDOMAIN_H

#include <cmath>
#include <vector>
#include <limits>
#include <iostream>
#include <cassert>

namespace test_heuristic_search{


class TestDomain_
{
public:    

    typedef int Cost;
    
    std::string description()
    {
        return "TestDomain_";
    }
    
    static Cost costMax()
    {
        return std::numeric_limits<Cost>::max();
    }
    
    static Cost costFactor()
    {
        return 100;
    }
    
    
    struct Point
    {
        int x,y;
    };
    
    struct State
    {
        int id;
        
        State(int _id = -1) : id(_id)
        {
            
        }
        
        bool operator==(State const& rhs) const
        {
            return id==rhs.id;
        }
        
        bool operator!=(State const& rhs) const
        {
            return id!=rhs.id;
        }
    };

    struct Action
    {
        Action(Cost cost = std::numeric_limits<Cost>::infinity()) : cost(cost)
        {
            
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
    
    TestDomain_(std::vector<State> _states,
        std::vector<std::vector<std::pair<int, Action> > > _successors,
        std::vector<Point> _points)
    {
        states = _states;
        successors_ = _successors;        
        points = _points;
        
        predecessors_.resize(successors_.size());
        
        for(int i=0; i<int(states.size()); ++i)
        {
            for(auto action_pair : successors_[i])
            {                
                predecessors_[action_pair.first].push_back(std::pair<int,Action>(i, action_pair.second));
            }            
        }
    }
    
    
    std::vector<std::pair<StateActionState,Cost> > mapUpdate(State const& current_state)
    {
        std::vector<std::pair<StateActionState,Cost> > changed_actions;
        
        return changed_actions;
    }
        
    int stateId(State const&state) const
    {
        if(state.id >=0 && state.id<static_cast<int>(states.size()))
            return state.id;
        
        return -1;
    }
    
    int size()
    {
        return states.size();
    }
    
    std::vector<StateActionState> getActions()
    {
        std::vector<StateActionState> _actions;
        
        for(int i=0; i<static_cast<int>(states.size()); ++i)
        {
            for(auto action_pair : successors_[i])
            {                

#ifndef NDEBUG                
                for(auto sas : _actions)
                {
                    assert(sas.from != states[i] || sas.to != states[action_pair.first]);
                }
#endif                
                _actions.push_back(
                    StateActionState(states[i], action_pair.second, states[action_pair.first]));
            }
            
        }
        
        return _actions;
    }
    
    std::pair<std::vector<StateActionState>, std::vector<StateActionState> > 
        getNeighborhood(State state)
    {
        std::vector<StateActionState> predecessors = getPredecessors(state); 
        std::vector<StateActionState> successors = getSuccessors(state);

        return std::pair<std::vector<StateActionState>, 
            std::vector<StateActionState> >(successors, predecessors);
    }
    
    std::vector<StateActionState> getSuccessors(State state)
    {
        std::vector<StateActionState> successors;
        
        for(auto succ : successors_[stateId(state)])
        {
            successors.push_back(StateActionState(state, succ.second, states[succ.first]));
        }
        
        return successors;
    }
    
    std::vector<StateActionState> getPredecessors(State state)
    {
        std::vector<StateActionState> predecessors;
        
        for(auto pred : predecessors_[stateId(state)])
        {
            predecessors.push_back(StateActionState(states[pred.first], pred.second, state));
        }
        
        return predecessors;
    }
    
    Cost heuristic(State from, State to)
    {
        return costFactor()*std::hypot(points[to.id].x - points[from.id].x,
                points[to.id].y - points[from.id].y);
    }
    
    Cost cost(State from, State to)
    {
        for(auto succ : successors_[from.id])
        {
            if(succ.first == to.id)
            {
                return succ.second.cost;
            }
            
        }
        
        return costMax();
        
    }
    
    void updateAction(StateActionState const& action)
    {
        for(auto &succ : successors_[action.from.id])
        {
            if(succ.first == action.to.id)
            {
                succ.second = action.action;
            }
            
        }
        
        for(auto &pred : predecessors_[action.to.id])
        {
            if(pred.first == action.from.id)
            {
                pred.second = action.action;
            }
            
        }
        
        assert(action.action.cost == cost(action.from, action.to));
    }
    
    static bool equal(Cost const& c1, Cost const& c2)
    {
        return c1==c2;
    }
    
private:
    
    std::vector<Point> points;
    std::vector<State> states;
    std::vector<std::vector<std::pair<int, Action> > > successors_;
    std::vector<std::vector<std::pair<int, Action> > > predecessors_;
};

class TestDomainTop;

// level 1
//  0__1__2__3__4__5
//     |__6__7__|

class TestDomain : public TestDomain_
{
public:
        
    TestDomain() : TestDomain_(
                    {{0},{1},{2},{3},{4},{5},{6},{7}},
            
                    {
                    {{1, Action(1*costFactor())}},                                                                 //0
                    {{0, Action(1*costFactor())}, {2, Action(1*costFactor())}, {6, Action(2*costFactor())}},     //1
                    {{1, Action(1*costFactor())}, {3, Action(1*costFactor())}},                                   //2
                    {{2, Action(1*costFactor())}, {4, Action(1*costFactor())}},                                   //3
                    {{3, Action(1*costFactor())}, {5, Action(1*costFactor())}, {7, Action(2*costFactor())}},     //4
                    {{4, Action(1*costFactor())}},                                                                 //5
                    {{1, Action(2*costFactor())}, {7, Action(1*costFactor())}},                                   //6
                    {{4, Action(2*costFactor())}, {6, Action(1*costFactor())}}                                   //7
                    },
                            
                    {{0,0},{0,1},{0,2},{0,3},{0,4},{0,5},{1,2},{1,3}})
    {
        
    }
                    
                    
    State remap(State abstract_state, TestDomainTop *abstract_domain)
    {
        static std::vector<int> id_remapping = {{1},{2},{4},{6}};
        
        return State(id_remapping[abstract_state.id]);
    }
    
};



// level 0
//  (0)__(1)____(2)
//   |___(3)_____|

//  (0__1)__(2__3)__(4__5)      ^
//      |___(6__7)___|          | abstract mapping


class TestDomainTop : public TestDomain_
{
public:
    
    TestDomainTop() : TestDomain_(
                    {{0},{1},{2},{3}},
            
                    {
                    {{1, Action(1*costFactor())}, {3, Action(2*costFactor())}},                                   //0
                    {{0, Action(1*costFactor())}, {2, Action(2*costFactor())}},                                   //1
                    {{1, Action(2*costFactor())}, {3, Action(3*costFactor())}},                                   //2
                    {{1, Action(2*costFactor())}, {2, Action(3*costFactor())}}                                    //3
                    },
                            
                    {{0,1},{0,2},{0,4},{1,2}})
    {
        
    }
                    
    typedef TestDomain_::State State;
    
    static State remap(State basic_state, TestDomain *basic_domain)
    {
        static std::vector<int> id_remapping = {{0},{0},{1},{1},{2},{2},{3},{3}};
        
        return State(id_remapping[basic_state.id]);
    }
    
    template<typename FROM_DOMAIN>
    struct RemappingFunctor
    {
        typedef typename FROM_DOMAIN::State From_t;
        
        RemappingFunctor(FROM_DOMAIN *from_domain) : 
            from_domain(from_domain)
        {
            
        }
        
        RemappingFunctor(RemappingFunctor const& orig) : 
            from_domain(orig.from_domain)
        {
            
        }
        
        RemappingFunctor(RemappingFunctor && orig) : 
            from_domain(orig.from_domain)
        {
            
        }
        
        RemappingFunctor<FROM_DOMAIN> &operator=(RemappingFunctor const& orig)
        {
            from_domain = orig.from_domain;
            return *this;
        }
        
        RemappingFunctor<FROM_DOMAIN> &operator=(RemappingFunctor && orig)
        {
            from_domain = orig.from_domain;
            return *this;
        }
        
        int operator()(From_t basic_state) const
        {
            return TestDomainTop::remap(basic_state, from_domain).id;
        }
        
        FROM_DOMAIN *from_domain;
        
    };
    
    template<typename FROM_DOMAIN>
    RemappingFunctor<FROM_DOMAIN> getRemappingFunctor(FROM_DOMAIN *from_domain)
    {
        return RemappingFunctor<FROM_DOMAIN>(from_domain);
    }
    
};



}//namespace test_heuristic_search


std::ostream& operator<<(std::ostream& stream, test_heuristic_search::TestDomain_::State const& state);

#endif /* TEST_TESTDOMAIN_H */

