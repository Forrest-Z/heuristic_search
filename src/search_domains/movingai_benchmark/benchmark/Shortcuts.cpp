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


#include <map>

#include <boost/lexical_cast.hpp>

#include <heuristic_search/AStar.h>
#include <heuristic_search/loggers/Logger.h>

#include "search_domains/movingai_benchmark/Domain.h"
#include "search_domains/movingai_benchmark/benchmark/Shortcuts.h"
#include "search_domains/movingai_benchmark/loader/ScenarioDirectoryLoader.h"
#include "search_domains/movingai_benchmark/loader/MapLoader.h"

namespace movingai_benchmark{

namespace benchmark{

std::map<int,int> bucket_counter;

DynamicExperiment initializeExperimentForShortcutsAndBarriers(
        loader::Experiment const& experiment, 
        int observation_range,
        bool shortcut)
{
    DynamicExperiment unknown_terrain_experiment;
    
    int W;
    int H;
    unsigned char *map;
       
    if(!movingai_benchmark::loader::loadMap(experiment.GetMapName(),W,H,map))
    {
        std::cerr << "Couldn't open file " << experiment.GetMapName() << std::endl;
        
        return unknown_terrain_experiment;
    }
    
    unknown_terrain_experiment.experiment = experiment;
    unknown_terrain_experiment.W = W;
    unknown_terrain_experiment.H = H;
    unknown_terrain_experiment.observation_range = observation_range;
    unknown_terrain_experiment.true_map = map;
    unknown_terrain_experiment.known_map = new unsigned char[W*H];
    std::memset(unknown_terrain_experiment.known_map, movingai_benchmark::UNKNOWN_TERRAIN, 
            W*H*sizeof(unsigned char));
    
    if(shortcut)
    {
        std::swap(unknown_terrain_experiment.true_map, unknown_terrain_experiment.known_map);
    }
        
    return unknown_terrain_experiment;
}


void runBenchmarkWithShortcuts(
    std::string hog_benchmarks_dir,
    std::string problems_name,
    std::string test_name,
    bool preinitialized, 
    int observation_range,        
    int center_bucket, 
    std::vector<std::function<heuristic_search::loggers::Log(DynamicExperiment const&)> > algorithms,
    int problems_count,
    std::function<int(std::vector<loader::Experiment> const&)> experiment_id_selector,
    std::function<heuristic_search::loggers::Log(DynamicExperiment const&)> problem_validator
    )
{    
    std::string cb_id = boost::lexical_cast<std::string>(center_bucket);   
    
    loader::ScenarioDirectoryLoader scenarios_barriers(
            hog_benchmarks_dir+"/problems_sb/" + problems_name + "/barriers/" + cb_id + ".scen",
            hog_benchmarks_dir);
    
                     
    auto result_barriers = heuristic_search::benchmark::runBenchmark<loader::Experiment, DynamicExperiment>(
            scenarios_barriers.experiments(), 
            std::bind(movingai_benchmark::benchmark::runAStarForDynamicExperimentWithShortcuts, 
                std::placeholders::_1, 
                heuristic_search::SearchingDirection::Backward, 
                preinitialized), 
            algorithms,
            problems_count, 
            experiment_id_selector, 
            std::bind(initializeExperimentForShortcutsAndBarriers, std::placeholders::_1, observation_range, false)); 
      
    
    loader::ScenarioDirectoryLoader scenarios_shortcuts(
            hog_benchmarks_dir+"/problems_sb/" + problems_name + "/shortcuts/" + cb_id + ".scen",
            hog_benchmarks_dir);
    
           
    auto result_shortcuts = heuristic_search::benchmark::runBenchmark<loader::Experiment, DynamicExperiment>(
            scenarios_shortcuts.experiments(), 
            std::bind(movingai_benchmark::benchmark::runAStarForDynamicExperimentWithShortcuts, 
                std::placeholders::_1, 
                heuristic_search::SearchingDirection::Backward, 
                preinitialized), 
            algorithms,
            problems_count, 
            experiment_id_selector, 
            std::bind(initializeExperimentForShortcutsAndBarriers, std::placeholders::_1, observation_range, true)); 
    
    
    auto logs = result_barriers.first;
    auto experiments = result_barriers.second;
    
    logs.insert(logs.end(), result_shortcuts.first.begin(), result_shortcuts.first.end());
    experiments.insert(experiments.end(), result_shortcuts.second.begin(), result_shortcuts.second.end());
    
    heuristic_search::benchmark::printSummary(logs, 4, 1);
    
    std::string preinit = (preinitialized)?"preinit":"uninit";    
    std::string observ = boost::lexical_cast<std::string>(observation_range); 
    
    heuristic_search::benchmark::writeResults(hog_benchmarks_dir, 
            test_name+"/"+problems_name + "/" + preinit + "_obs" + observ +"_cb"+cb_id,
            experiments, logs, 4, 1);
}

void runBenchmarkWithShortcuts2(
    std::string hog_benchmarks_dir,
    std::string problems_name,
    std::string test_name,
    bool preinitialized, 
    int observation_range,   
    std::vector<std::function<heuristic_search::loggers::Log(DynamicExperiment const&)> > algorithms,
    int problems_count,
    std::function<heuristic_search::loggers::Log(DynamicExperiment const&)> problem_validator
    )
{    
    bucket_counter.clear();
        
    loader::ScenarioDirectoryLoader scenarios(
            hog_benchmarks_dir+"/problems/" + problems_name,
            hog_benchmarks_dir);
    
    
    if(scenarios.experiments().empty())
    {
        return;
    }
     
    movingai_benchmark::benchmark::RandomShortcutExperimentSelector problem_selector_barriers;
                     
    auto result_barriers = heuristic_search::benchmark::runBenchmark<loader::Experiment, DynamicExperiment>(
            scenarios.experiments(), 
            std::bind(movingai_benchmark::benchmark::runAStarForDynamicExperimentWithShortcuts, 
                std::placeholders::_1, 
                heuristic_search::SearchingDirection::Backward, 
                preinitialized), 
            algorithms,
            problems_count/2, 
            problem_selector_barriers, 
            std::bind(initializeExperimentForShortcutsAndBarriers, std::placeholders::_1, observation_range, false)); 
          
    movingai_benchmark::benchmark::RandomShortcutExperimentSelector problem_selector_shortcuts;
           
    auto result_shortcuts = heuristic_search::benchmark::runBenchmark<loader::Experiment, DynamicExperiment>(
            scenarios.experiments(), 
            std::bind(movingai_benchmark::benchmark::runAStarForDynamicExperimentWithShortcuts, 
                std::placeholders::_1, 
                heuristic_search::SearchingDirection::Backward, 
                preinitialized), 
            algorithms,
            problems_count/2, 
            problem_selector_shortcuts, 
            std::bind(initializeExperimentForShortcutsAndBarriers, std::placeholders::_1, observation_range, true)); 
    
    
    auto logs = result_barriers.first;
    auto experiments = result_barriers.second;
    
    logs.insert(logs.end(), result_shortcuts.first.begin(), result_shortcuts.first.end());
    experiments.insert(experiments.end(), result_shortcuts.second.begin(), result_shortcuts.second.end());
    
    heuristic_search::benchmark::printSummary(logs, 4, 1);
    
    std::string preinit = (preinitialized)?"preinit":"uninit";    
    std::string observ = boost::lexical_cast<std::string>(observation_range); 
    
    heuristic_search::benchmark::writeResults(hog_benchmarks_dir, 
            test_name+"/"+problems_name + "/" + problems_name + "_" + preinit + "_obs" + observ,
            experiments, logs, 4, 1);
}

heuristic_search::loggers::Log runAStarForDynamicExperimentWithShortcuts(
        DynamicExperiment const& experiment,
        heuristic_search::SearchingDirection direction, 
        bool preinitialized)
{    
    Domain domain(experiment.true_map, nullptr, experiment.W,experiment.H,0,1);
    
    Domain::State start(experiment.experiment.GetStartX(), experiment.experiment.GetStartY());
    Domain::State goal(experiment.experiment.GetGoalX(), experiment.experiment.GetGoalY());
              
    
    using namespace heuristic_search;
    
    typedef SearchAlgorithmBegin<            
            loggers::Logger<
            SearchLoop<
            AStar<
            HeuristicSearch<
            StdOpenList<
            StdSearchSpace<
        SearchAlgorithmEnd<Domain> > > > > > > >::Algorithm_t AStar_t;
                
    
    AStar_t algorithm(domain, direction, preinitialized);    
    algorithm.search(start, goal);
        
    Domain domain_known_map(experiment.known_map, nullptr, experiment.W,experiment.H,0,1);    
    AStar_t algorithm_known_map(domain_known_map, direction, preinitialized);    
    algorithm_known_map.search(start, goal);
    
    if(!algorithm.log.success || !algorithm_known_map.log.success)
    {
        return heuristic_search::loggers::Log();
    }
        
    algorithm.log.path_cost -= algorithm_known_map.log.path_cost;
    
    int bucket = int(algorithm.log.path_cost/4);
    
    if(bucket_counter[bucket]>=200)
    {
        algorithm.log.success = 0;
    }
    else
    {
        bucket_counter[bucket]++;
    }
    
    return algorithm.log;
}

}//namespace benchmark
}//namespace movingai_benchmark