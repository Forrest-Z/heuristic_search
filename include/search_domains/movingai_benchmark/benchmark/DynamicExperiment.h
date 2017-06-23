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


#ifndef MOVINGAI_BENCHMARK_DYNAMIC_EXPERIMENT_H
#define MOVINGAI_BENCHMARK_DYNAMIC_EXPERIMENT_H

#include <functional>

#include "heuristic_search/loggers/Log.h"
#include "heuristic_search/HeuristicSearch.h"

#include "search_domains/movingai_benchmark/Domain.h"
#include "search_domains/movingai_benchmark/loader/ScenarioLoader.h"

namespace movingai_benchmark{
namespace benchmark{

struct DynamicExperimentParams
{
    DynamicExperimentParams()
    {
        observation_range = 0;
        preinitialized = false;
        undirected = false;
    }
    
    int observation_range;
    bool preinitialized;
    bool undirected;
};

struct DynamicExperiment : public DynamicExperimentParams
{
    DynamicExperiment()
    {
        W = 0;
        H = 0;
        known_map = nullptr;
        true_map = nullptr;
    }
    
    virtual ~DynamicExperiment()
    {
        delete [] known_map;
        delete [] true_map;
    }
    
    loader::Experiment experiment;
    unsigned char *known_map;
    unsigned char *true_map;
    int W,H;
};

heuristic_search::loggers::Log runAStarForDynamicExperiment(
        DynamicExperiment const& experiment, 
        heuristic_search::SearchingDirection direction = 
            heuristic_search::SearchingDirection::Forward, 
        bool preinitialized = false);

template<typename Algorithm_t, heuristic_search::SearchingDirection search_direction>
heuristic_search::loggers::Log runDynamicExperiment(
    movingai_benchmark::benchmark::DynamicExperiment const& experiment)
{        
    movingai_benchmark::Domain domain(experiment.known_map, experiment.true_map, 
            experiment.W, experiment.H, experiment.observation_range,1);
    
    Algorithm_t algorithm(domain, search_direction, experiment.preinitialized, 
            experiment.undirected);
        
    algorithm.main(
        movingai_benchmark::Domain::State(experiment.experiment.GetStartX(),
                                        experiment.experiment.GetStartY()),
        movingai_benchmark::Domain::State(experiment.experiment.GetGoalX(), 
                                        experiment.experiment.GetGoalY())
    );
        
    heuristic_search::loggers::Log log = algorithm.result();
        
    return log;
}


heuristic_search::loggers::Log runDStarExtraLite(
    movingai_benchmark::benchmark::DynamicExperiment const& experiment);

std::function<heuristic_search::loggers::Log(
        movingai_benchmark::benchmark::DynamicExperiment const& )> 
    getDynamicAlgorithm(std::string const& name);

std::vector<
    std::function<heuristic_search::loggers::Log(
        movingai_benchmark::benchmark::DynamicExperiment const& )> > 
    getDynamicAlgorithms(
        std::vector<std::string> const& names,
        std::function<
            std::function<
                heuristic_search::loggers::Log(
                    movingai_benchmark::benchmark::DynamicExperiment const& 
                )
            >(std::string const&)
        > algorithm_selector = getDynamicAlgorithm);

}//namespace benchmark
}//namespace movingai_benchmark

#endif /* MOVINGAI_BENCHMARK_DYNAMIC_EXPERIMENT_H */

