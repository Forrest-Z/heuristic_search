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

#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <string>
#include <vector>
#include <functional>
#include <iostream>
#include <fstream>
#include <cassert>
#include <ctime>
#include <iomanip>
#include <chrono>

#include <boost/filesystem.hpp>

#include "heuristic_search/loggers/Log.h"

namespace heuristic_search{

namespace benchmark{

template<typename EXPERIMENT>
struct RandomExperimentSelector
{
    int operator()(std::vector<EXPERIMENT> const& experiments)
    {
        if(experiments.empty())
        {
            return -1;
        }
        
        if(used_experiment.size() < experiments.size())
        {
            used_experiment.resize(experiments.size(), false);
        }

        if(std::find(used_experiment.begin(), used_experiment.end(), false) == used_experiment.end())
        {
            return -1;
        }
        
        int id = -1;
        
        do
        {
            id = std::rand()%experiments.size();
        }
        while(used_experiment[id]); 
        
        used_experiment[id] = true;
                        
        return id;
    }
    
    std::vector<bool> used_experiment;
};

template<typename Problem, typename InitializedProblem>
InitializedProblem defaultProblemInitialize(Problem  const&problem)
{
    return problem;
}

template<typename Problem, typename InitializedProblem=Problem>
std::pair<std::vector<loggers::Log>,  Problem > runBenchmarkProblem(
        Problem problem, 
        std::function<loggers::Log(InitializedProblem const&)> problem_validator,
        std::vector<std::function<loggers::Log(InitializedProblem const&)> > algorithms,
        std::function<InitializedProblem(Problem const&)> problem_initializer = 
            defaultProblemInitialize<Problem,InitializedProblem>)
{
    InitializedProblem initialized_problem = problem_initializer(problem);
        
    std::cout << problem << std::endl;

    loggers::Log validation_log = problem_validator(initialized_problem);

    if(!validation_log.success)
    {
        std::cout << "invalid!" << std::endl;

    }
    else
    {
        loggers::Log::writeHeader(std::cout);
        std::cout << std::endl;

        std::cout << validation_log << std::endl;

        std::vector<loggers::Log> problem_logs;

        problem_logs.push_back(validation_log);

        bool all_succeed = true;

        for(auto algorithm : algorithms)
        {
            loggers::Log log = algorithm(initialized_problem);

            assert(!log.success || validation_log.path_cost <= log.path_cost + 1.0e-3);

            problem_logs.push_back(log);

            std::cout << log << std::endl;

            if(!log.success)
            {
                all_succeed = false;
                break;
            }
        }
        
        std::cout << std::endl;

        if(all_succeed)
        {       
            return std::pair<std::vector<loggers::Log>, Problem>(problem_logs, problem); 
        }
    }
    
    return std::pair<std::vector<loggers::Log>, Problem>(); 
}

template<typename Problem, typename InitializedProblem=Problem>
std::pair<std::vector<std::vector<loggers::Log> >,  std::vector<Problem> > runBenchmark(
        std::vector<Problem> problems, 
        std::function<loggers::Log(InitializedProblem const&)> problem_validator,
        std::vector<std::function<loggers::Log(InitializedProblem const&)> > algorithms,
    int problems_count,
    std::function<int(std::vector<Problem> const&)> problem_id_selector = 
        std::function<int(std::vector<Problem> const&)>(),
    std::function<InitializedProblem(Problem const&)> problem_initializer = 
        defaultProblemInitialize<Problem,InitializedProblem>)
{
    std::vector<std::vector<loggers::Log> > logs;
        
    
    int problem_id;
    if(problem_id_selector)
    {
        problem_id = problem_id_selector(problems);
    }
    else
    {
        problem_id = 0;
    }
    
    int valid_problems = 0;
    
    std::vector<Problem> used_problems;
    
    while(problem_id > -1 
            && problem_id < static_cast<int>(problems.size())
            && (valid_problems < problems_count || problems_count<=0))
    {               
        Problem problem = problems[problem_id];
        InitializedProblem initialized_problem = problem_initializer(problem);
        
        std::cout << "["<< valid_problems+1 << "] " << problem << std::endl;
        
        loggers::Log validation_log = problem_validator(initialized_problem);
        
        if(!validation_log.success)
        {
            std::cout << "invalid!" << std::endl;
            
        }
        else
        {
            loggers::Log::writeHeader(std::cout);
            std::cout << std::endl;

            std::cout << validation_log << std::endl;

            std::vector<loggers::Log> problem_logs;

            problem_logs.push_back(validation_log);
            
            bool all_succeed = true;

            for(auto algorithm : algorithms)
            {
                loggers::Log log = algorithm(initialized_problem);
                
                assert(!log.success || validation_log.path_cost <= log.path_cost + 1.0e-3);
                
                problem_logs.push_back(log);

                std::cout << log << std::endl;
                
                if(!log.success)
                {
                    all_succeed = false;
                    break;
                }
            }
        
            if(all_succeed)
            {                
                ++valid_problems;                
                used_problems.push_back(problem);
                logs.push_back(problem_logs);
            }

            std::cout << std::endl;
        }
        
        if(problem_id_selector)
        {
            problem_id = problem_id_selector(problems);
        }
        else
        {
            problem_id++;
        }
    }
    
    
    return std::pair<std::vector<std::vector<loggers::Log> >,  std::vector<Problem> >(logs, used_problems);
}

std::vector<loggers::Log> mean(std::vector<std::vector<loggers::Log> > logs, int min_problems_number = 1);
std::vector<std::vector<loggers::Log> > meanBuckets(std::vector<std::vector<loggers::Log> > logs, 
        double bucket_cost_range, int min_problems_number = 1);

void writeBucketLogsToFile(std::string filename,  
        std::vector<std::vector<loggers::Log> > buckets);

template<typename Problem>
void writeLogsToFile(std::string filename, std::vector<Problem> problems, 
        std::vector<std::vector<loggers::Log> > logs)
{    
    assert(problems.size() == logs.size());
    
    boost::filesystem::path path(filename);
    
    if(!boost::filesystem::exists(path.parent_path()))
    {
        if(!boost::filesystem::create_directories(path.parent_path()))
        {
            std::cerr << "Couldn't create directory " << path.parent_path() << std::endl;
        
            return;
        }
    }
    
    std::ofstream file(filename.c_str());
    
    if(!file.good())
    {
        std::cerr << "Couldn't open file " << filename << std::endl;
        
        return;
    }
    
    for(int i=0; i<static_cast<int>(problems.size()); ++i)
    {
        file << problems[i];
        
        for(auto log : logs[i])
        {
            file << log;
        }
        
        file << std::endl;
    }
    
    
    std::cout << "Full logs written to \n" << filename << std::endl << std::endl;
    
}
        
void writeSummary(std::ostream &ostream, std::vector<std::vector<loggers::Log> > logs,
        double bucket_cost_range=4.0, int min_problems_number=1);

void writeSummaryToFile(std::string filename,  
        std::vector<std::vector<loggers::Log> > logs,
        double bucket_cost_range=4.0, int min_problems_number=1);
        
void printSummary(std::vector<std::vector<loggers::Log> > logs,
        double bucket_cost_range=4.0, int min_problems_number=1);

template<typename Problem>
void writeResults(std::string directory, std::string experiment_name,
        std::vector<Problem> problems, std::vector<std::vector<loggers::Log> > logs,
        double bucket_cost_range=4.0, int min_problems_number=1)
{
    std::time_t tt = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now());  
    auto tm = *std::localtime(&tt);
    
    char buffer[80];
    std::strftime(buffer,80,"_%F_%H-%M-%S",&tm);
    std::string timestamp(buffer);
    
    std::string filename = directory+"/results/"+experiment_name+timestamp;
    
    benchmark::writeLogsToFile(
            filename+".result", 
            problems, logs);
    
    benchmark::writeBucketLogsToFile(
            filename+"_buckets.result", 
            benchmark::meanBuckets(logs, 4, 1));
    
    benchmark::writeSummaryToFile(
            filename+"_summary.result", 
            logs, 4, 1);
}

}//namespace benchmark

}//namespace heuristic_search


#endif /* BENCHMARK_H */

