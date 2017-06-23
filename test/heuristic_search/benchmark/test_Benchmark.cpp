/* heuristic_search library
 *
 * Copyright (c) 2016, 
 * Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>, 
 * Warsaw University of Technology.
 * All rights reserved.
 *  
 */



#include <gtest/gtest.h>

#include "heuristic_search/benchmark/Benchmark.h"


#include "../../heuristic_search/test_TestDomain.h"



namespace test_heuristic_search{
struct Problem;
}

std::ostream &operator<<(std::ostream &stream, test_heuristic_search::Problem const& problem);

namespace test_heuristic_search{


struct Problem
{
    TestDomain::State start;
    TestDomain::State goal;
};


    
TEST(test_Benchmark, run)
{        
    std::vector<Problem> problems = {
        {{0},{1}}, 
        {{2},{3}}, 
        {{4},{5}} 
    };
    
    
    std::function<heuristic_search::loggers::Log(Problem const&)> problem_validator = 
        [](Problem problem) -> heuristic_search::loggers::Log
        {
            heuristic_search::loggers::Log log;
            log.success = 1;
            
            return log;   
        };
    
    
    
    std::vector<std::function<heuristic_search::loggers::Log(Problem const&)> > algorithms;
    
    std::function<heuristic_search::loggers::Log(Problem)> algorithm_1 = 
        [](Problem problem) -> heuristic_search::loggers::Log
        {
            heuristic_search::loggers::Log log;
            
            log.search_steps = problem.start.id;
            log.success = 1;
            
            return log;   
        };
    
    std::function<heuristic_search::loggers::Log(Problem const&)> algorithm_2 = 
        [](Problem problem) -> heuristic_search::loggers::Log
        {
            heuristic_search::loggers::Log log;
            
            log.search_steps = problem.goal.id;
            log.success = 1;
            
            return log;   
        };
    
    algorithms.push_back(algorithm_1);
    algorithms.push_back(algorithm_2);
        
    auto result = heuristic_search::benchmark::runBenchmark(problems, problem_validator, algorithms,
        problems.size()); 

    auto logs = result.first;
    
    ASSERT_EQ(problems.size(), logs.size());
    ASSERT_EQ(algorithms.size()+1, logs[0].size());
    EXPECT_EQ(0, logs[0][1].search_steps);
    EXPECT_EQ(1, logs[0][2].search_steps);
    EXPECT_EQ(2, logs[1][1].search_steps);
    EXPECT_EQ(3, logs[1][2].search_steps);
    EXPECT_EQ(4, logs[2][1].search_steps);
    EXPECT_EQ(5, logs[2][2].search_steps);
}
    
    
}

std::ostream &operator<<(std::ostream &stream, test_heuristic_search::Problem const& problem)
{
    stream << problem.start << " " << problem.goal;
    
    return stream;
}

