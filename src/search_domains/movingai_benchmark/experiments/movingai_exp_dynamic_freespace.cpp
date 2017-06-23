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


#include <boost/program_options.hpp>

#include <heuristic_search/AStar.h>
#include <heuristic_search/DStarExtraLite.h>
#include <heuristic_search/DStarExtraLiteNoKm.h>
#include <heuristic_search/DStarExtraLiteUndirected.h>
#include <heuristic_search/DStarLiteOptimized.h>
#include <heuristic_search/DStarLite.h>
#include <heuristic_search/DStarMain.h>
#include "heuristic_search/MPGAAStar.h"
#include <heuristic_search/IncrementalSearch.h>
#include <heuristic_search/loggers/Logger.h>
#include <heuristic_search/loggers/DStarLogger.h>
#include <heuristic_search/loggers/MPGAALogger.h>
#include <heuristic_search/benchmark/Benchmark.h>

#include <search_domains/movingai_benchmark/Domain.h>
#include <search_domains/movingai_benchmark/loader/MapLoader.h>
#include <search_domains/movingai_benchmark/loader/ScenarioLoader.h>
#include "search_domains/movingai_benchmark/loader/ScenarioDirectoryLoader.h"
#include "search_domains/movingai_benchmark/benchmark/Benchmark.h"
#include "search_domains/movingai_benchmark/benchmark/Freespace.h"
#include "search_domains/movingai_benchmark/ProgramOptions.h"


heuristic_search::loggers::Log runDStarExtraLite(
    movingai_benchmark::benchmark::DynamicExperiment const& experiment, bool preinitialized, bool undirected)
{    
    typedef heuristic_search::SearchAlgorithmBegin<    
            heuristic_search::loggers::DStarLogger<     
            heuristic_search::DStarMain<
            heuristic_search::SearchLoop<
            heuristic_search::DStarExtraLite<
            heuristic_search::IncrementalSearch_SearchStep<
            heuristic_search::AStar<
            heuristic_search::IncrementalSearch_Initialize<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > > > > >::Algorithm_t Algorithm_t;
    
        
    movingai_benchmark::Domain domain(experiment.known_map, experiment.true_map, 
            experiment.W, experiment.H, experiment.observation_range,1);
    
    Algorithm_t algorithm(domain, heuristic_search::SearchingDirection::Backward, preinitialized, undirected);
        
    algorithm.main(
        movingai_benchmark::Domain::State(experiment.experiment.GetStartX(), experiment.experiment.GetStartY()),
        movingai_benchmark::Domain::State(experiment.experiment.GetGoalX(), experiment.experiment.GetGoalY()));
                
    heuristic_search::loggers::Log log = algorithm.result();
        
    return log;
}

heuristic_search::loggers::Log runDStarExtraLiteUndirected(
    movingai_benchmark::benchmark::DynamicExperiment const& experiment, bool preinitialized, bool undirected)
{    
    typedef heuristic_search::SearchAlgorithmBegin<    
            heuristic_search::loggers::DStarLogger<     
            heuristic_search::DStarMain<
            heuristic_search::SearchLoop<
            heuristic_search::DStarExtraLiteUndirected_Reinitialize<
            heuristic_search::DStarExtraLite_Base<
            heuristic_search::IncrementalSearch_SearchStep<
            heuristic_search::AStar<
            heuristic_search::IncrementalSearch_Initialize<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > > > > > >::Algorithm_t Algorithm_t;
    
        
    movingai_benchmark::Domain domain(experiment.known_map, experiment.true_map, 
            experiment.W, experiment.H, experiment.observation_range,1);
    
    Algorithm_t algorithm(domain, heuristic_search::SearchingDirection::Backward, preinitialized, undirected);
        
    algorithm.main(
        movingai_benchmark::Domain::State(experiment.experiment.GetStartX(), experiment.experiment.GetStartY()),
        movingai_benchmark::Domain::State(experiment.experiment.GetGoalX(), experiment.experiment.GetGoalY()));
                
    heuristic_search::loggers::Log log = algorithm.result();
        
    return log;
}

heuristic_search::loggers::Log runDStarExtraLiteNoKm(
    movingai_benchmark::benchmark::DynamicExperiment const& experiment, bool preinitialized, bool undirected)
{    
    typedef heuristic_search::SearchAlgorithmBegin<    
            heuristic_search::loggers::DStarLogger<        
            heuristic_search::DStarMain<
            heuristic_search::SearchLoop<
            heuristic_search::DStarExtraLiteNoKm_Reinitialize<
            heuristic_search::DStarExtraLite_Base<
            heuristic_search::IncrementalSearch_SearchStep<
            heuristic_search::AStar<
            heuristic_search::IncrementalSearch_Initialize<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > > > > > >::Algorithm_t Algorithm_t;
    
        
    movingai_benchmark::Domain domain(experiment.known_map, experiment.true_map, 
            experiment.W, experiment.H, experiment.observation_range,1);
    
    Algorithm_t algorithm(domain, heuristic_search::SearchingDirection::Backward, preinitialized, undirected);
        
    algorithm.main(
        movingai_benchmark::Domain::State(experiment.experiment.GetStartX(), experiment.experiment.GetStartY()),
        movingai_benchmark::Domain::State(experiment.experiment.GetGoalX(), experiment.experiment.GetGoalY()));
                
    heuristic_search::loggers::Log log = algorithm.result();
        
    return log;
}

heuristic_search::loggers::Log runDStarLiteOptimized(
    movingai_benchmark::benchmark::DynamicExperiment const& experiment, bool preinitialized, bool undirected)
{    
    typedef heuristic_search::SearchAlgorithmBegin<    
            heuristic_search::loggers::DStarLogger<      
            heuristic_search::DStarMain<
            heuristic_search::SearchLoop<
            heuristic_search::DStarLiteOptimized<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > >::Algorithm_t Algorithm_t;
    
    movingai_benchmark::Domain domain(experiment.known_map, experiment.true_map, 
            experiment.W, experiment.H, experiment.observation_range,1);
    
    Algorithm_t algorithm(domain, heuristic_search::SearchingDirection::Backward, preinitialized, undirected);
        
    algorithm.main(
        movingai_benchmark::Domain::State(experiment.experiment.GetStartX(), experiment.experiment.GetStartY()),
        movingai_benchmark::Domain::State(experiment.experiment.GetGoalX(), experiment.experiment.GetGoalY()));
        
    heuristic_search::loggers::Log log = algorithm.result();
        
    return log;
}

heuristic_search::loggers::Log runDStarLite(
    movingai_benchmark::benchmark::DynamicExperiment const& experiment, bool preinitialized, bool undirected)
{    
    typedef heuristic_search::SearchAlgorithmBegin<    
            heuristic_search::loggers::DStarLogger<     
            heuristic_search::DStarMain<
            heuristic_search::SearchLoop<
            heuristic_search::DStarLite<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > >::Algorithm_t Algorithm_t;
    
    movingai_benchmark::Domain domain(experiment.known_map, experiment.true_map, 
            experiment.W, experiment.H, experiment.observation_range,1);
    
    Algorithm_t algorithm(domain, heuristic_search::SearchingDirection::Backward, preinitialized, undirected);
        
    algorithm.main(
        movingai_benchmark::Domain::State(experiment.experiment.GetStartX(), experiment.experiment.GetStartY()),
        movingai_benchmark::Domain::State(experiment.experiment.GetGoalX(), experiment.experiment.GetGoalY()));
        
    heuristic_search::loggers::Log log = algorithm.result();
        
    return log;
}

heuristic_search::loggers::Log runMPGAAStar(
    movingai_benchmark::benchmark::DynamicExperiment const& experiment, bool preinitialized, bool undirected)
{    
    typedef heuristic_search::SearchAlgorithmBegin<    
            heuristic_search::loggers::MPGAALogger<        
            heuristic_search::DStarMain<
            heuristic_search::MPGAAStar<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > >::Algorithm_t Algorithm_t;
    
    movingai_benchmark::Domain domain(experiment.known_map, experiment.true_map, 
            experiment.W, experiment.H, experiment.observation_range,1);
    
    Algorithm_t algorithm(domain, heuristic_search::SearchingDirection::Forward, preinitialized, undirected);
        
    algorithm.main(
        movingai_benchmark::Domain::State(experiment.experiment.GetStartX(), experiment.experiment.GetStartY()),
        movingai_benchmark::Domain::State(experiment.experiment.GetGoalX(), experiment.experiment.GetGoalY()));
        
    heuristic_search::loggers::Log log = algorithm.result();
        
    return log;
}


/*
 * 
 */
int main(int argc, char** argv) 
{
    namespace po = boost::program_options;
            
    po::options_description visible_ops;    
    visible_ops.add(movingai_benchmark::generalOptions())
               .add(movingai_benchmark::benchmarkOptions())
               .add(movingai_benchmark::dynamicExperimentOptions())
               .add(movingai_benchmark::configOptions());
    
    po::options_description basic_ops;
    basic_ops.add(movingai_benchmark::generalOptions())
                .add(movingai_benchmark::configOptions()); 
    
    try
    {    
        po::variables_map basic_vm;        
        po::store(
            po::basic_command_line_parser<char>(argc,argv).options(basic_ops).allow_unregistered().run(), 
            basic_vm);
        po::notify(basic_vm);

        if(basic_vm.count("help"))
        {
            std::cout << visible_ops << "\n";
            return 1;
        }

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, visible_ops), vm);

        if(basic_vm.count("config-file"))
        {
            std::string config_filename = basic_vm["config-file"].as<std::string>();
            std::cout << "Loading params from file: " << config_filename << "\n";
            po::store(po::parse_config_file<char>(config_filename.c_str(), visible_ops), vm);
        }

        po::notify(vm);
       
                        
        int observation_range = vm["observation-range"].as<int>();
        bool preinitialized = vm["preinitialized"].as<bool>();
        bool undirected = vm["undirected"].as<bool>();
        
        std::string hog_benchmarks_dir = vm["benchmark-dir"].as<std::string>();
        std::string problems_name = vm["problems-path"].as<std::string>();    
        std::string test_name = vm["test-name"].as<std::string>();
        int problems_count = vm["problems-number"].as<int>();
                        
        std::vector<std::function<heuristic_search::loggers::Log(
        movingai_benchmark::benchmark::DynamicExperiment const& )> > algorithms;
        algorithms.push_back(std::bind(runDStarExtraLite, std::placeholders::_1, preinitialized, undirected));
        algorithms.push_back(std::bind(runDStarExtraLiteUndirected, std::placeholders::_1, preinitialized, undirected));
        algorithms.push_back(std::bind(runDStarExtraLiteNoKm, std::placeholders::_1, preinitialized, undirected));
        algorithms.push_back(std::bind(runDStarLiteOptimized, std::placeholders::_1, preinitialized, undirected));
        algorithms.push_back(std::bind(runDStarLite, std::placeholders::_1, preinitialized, undirected));
        algorithms.push_back(std::bind(runMPGAAStar, std::placeholders::_1, preinitialized, undirected));


        movingai_benchmark::benchmark::runFreespaceBenchmark(
                hog_benchmarks_dir,
                problems_name,
                test_name,
                preinitialized,
                undirected,
                observation_range,
                algorithms,
                problems_count,
                movingai_benchmark::benchmark::RandomExperimentSelector(),
                std::bind(movingai_benchmark::benchmark::runAStarForDynamicExperiment, 
                    std::placeholders::_1, 
                    heuristic_search::SearchingDirection::Backward, preinitialized));

    }
    catch(boost::program_options::error const&e)
    {
        std::cout << e.what() << "\nCheck --help\n";
        return 1;
    }
    
    return 0;
}

