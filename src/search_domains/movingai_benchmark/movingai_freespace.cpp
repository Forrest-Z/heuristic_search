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


#include <cstdlib>
#include <iostream>
#include <boost/program_options.hpp>

#include "heuristic_search/benchmark/Benchmark.h"
#include "search_domains/movingai_benchmark/benchmark/Freespace.h"
#include "search_domains/movingai_benchmark/ProgramOptions.h"
#include "search_domains/movingai_benchmark/debug_cv/DynamicAlgorithms.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) 
{
    namespace po = boost::program_options;
            
    po::options_description visible_ops;    
    visible_ops.add(movingai_benchmark::generalOptions())
               .add(movingai_benchmark::algorithmsOptions())
               .add(movingai_benchmark::singleProblemOptions())
               .add(movingai_benchmark::dynamicExperimentOptions())
               .add(movingai_benchmark::visualizationOptions())
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

        std::vector<std::string > algorithm_names = vm["algorithms"].as<std::vector<std::string>>();
        
        movingai_benchmark::loader::Experiment problem;    
        problem.SetMap(vm["map-file"].as<std::string>());
        problem.SetStartX(vm["start.x"].as<int>());
        problem.SetStartY(vm["start.y"].as<int>());
        problem.SetGoalX(vm["goal.x"].as<int>());
        problem.SetGoalY(vm["goal.y"].as<int>());
                
        movingai_benchmark::benchmark::DynamicExperimentParams dynamic_experiment_params;
        dynamic_experiment_params.observation_range = vm["observation-range"].as<int>();
        dynamic_experiment_params.preinitialized = vm["preinitialized"].as<bool>();
        dynamic_experiment_params.undirected = vm["undirected"].as<bool>();
                
        bool visualization = vm["visualization"].as<bool>();
        movingai_benchmark::debug_cv::VisualizationConfig vis_config;
        vis_config.scale = vm["vis-scale"].as<double>();
        vis_config.delay = vm["vis-delay"].as<int>();
        vis_config.hide_text = vm["vis-hide-text"].as<bool>();
        
        std::function<heuristic_search::loggers::Log(
            movingai_benchmark::benchmark::DynamicExperiment const& )> problem_validator;
        
        std::vector<
            std::function<heuristic_search::loggers::Log(
                movingai_benchmark::benchmark::DynamicExperiment const& )> > algorithms;
        
        if(visualization)
        {
            problem_validator = 
                std::bind(movingai_benchmark::debug_cv::runAStarForDynamicExperiment, 
                std::placeholders::_1, 
                heuristic_search::SearchingDirection::Backward, 
                "A* - validation", vis_config);
        
            algorithms = movingai_benchmark::benchmark::getDynamicAlgorithms(                
                    algorithm_names, 
                    std::bind(
                        movingai_benchmark::debug_cv::getDynamicAlgorithm,
                        std::placeholders::_1,
                        vis_config
                    )
                );
        }
        else
        {
            problem_validator = 
                std::bind(movingai_benchmark::benchmark::runAStarForDynamicExperiment, 
                std::placeholders::_1, 
                heuristic_search::SearchingDirection::Backward, dynamic_experiment_params.preinitialized);
        
            algorithms = movingai_benchmark::benchmark::getDynamicAlgorithms(algorithm_names);
        }
        
            
        
        heuristic_search::benchmark::runBenchmarkProblem<
                movingai_benchmark::loader::Experiment, 
                movingai_benchmark::benchmark::DynamicExperiment>(
            problem, 
            problem_validator, 
            algorithms,
            std::bind(movingai_benchmark::benchmark::initializeFreespaceExperiment, std::placeholders::_1, 
                dynamic_experiment_params)); 

    }
    catch(boost::program_options::error const&e)
    {
        std::cout << e.what() << "\nCheck --help\n";
        return 1;
    }

    return 0;
}

