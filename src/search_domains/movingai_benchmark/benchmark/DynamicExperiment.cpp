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


#include "search_domains/movingai_benchmark/benchmark/DynamicExperiment.h"


#include <heuristic_search/AStar.h>
#include <heuristic_search/DStarExtraLite.h>
#include <heuristic_search/DStarExtraLiteNoKm.h>
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

namespace movingai_benchmark{
namespace benchmark{

using namespace heuristic_search;
using namespace movingai_benchmark;

heuristic_search::loggers::Log runAStarForDynamicExperiment(
        DynamicExperiment const& experiment, 
        heuristic_search::SearchingDirection direction, bool preinitialized)
{
    typedef SearchAlgorithmBegin<            
            loggers::Logger<
            SearchLoop<
            AStar<
            HeuristicSearch<
            StdOpenList<
            StdSearchSpace<
        SearchAlgorithmEnd<Domain> > > > > > > >::Algorithm_t AStar_t;
    
            
    Domain domain(experiment.true_map, nullptr, experiment.W,experiment.H,0,1);
    
    AStar_t algorithm(domain, direction, preinitialized);
    
    Domain::State start(experiment.experiment.GetStartX(), experiment.experiment.GetStartY());
    Domain::State goal(experiment.experiment.GetGoalX(), experiment.experiment.GetGoalY());
    algorithm.search(start, goal);
        
    return algorithm.log;
}


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
    heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > > > > >::Algorithm_t DStarExraLite_t;
   

   
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
    heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > > > > > >::Algorithm_t DStarExtraLiteNoKm_t;
    
        

typedef heuristic_search::SearchAlgorithmBegin<    
        heuristic_search::loggers::DStarLogger<      
        heuristic_search::DStarMain<
        heuristic_search::SearchLoop<
        heuristic_search::DStarLiteOptimized<
        heuristic_search::HeuristicSearch<
        heuristic_search::StdOpenList<
        heuristic_search::StdSearchSpace<
    heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > >::Algorithm_t DStarLiteOptimized_t;
    


typedef heuristic_search::SearchAlgorithmBegin<    
        heuristic_search::loggers::DStarLogger<     
        heuristic_search::DStarMain<
        heuristic_search::SearchLoop<
        heuristic_search::DStarLite<
        heuristic_search::HeuristicSearch<
        heuristic_search::StdOpenList<
        heuristic_search::StdSearchSpace<
    heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > >::Algorithm_t DStarLite_t;
 


typedef heuristic_search::SearchAlgorithmBegin<    
            heuristic_search::loggers::MPGAALogger<        
            heuristic_search::DStarMain<
            heuristic_search::MPGAAStar<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > >::Algorithm_t MPGAAStar_t;

std::function<heuristic_search::loggers::Log(
        movingai_benchmark::benchmark::DynamicExperiment const& )> getDynamicAlgorithm(std::string const& name)
{
    if(name=="D*ExtraLite")
    {
        return runDynamicExperiment<DStarExraLite_t, 
                    heuristic_search::SearchingDirection::Backward>;
    }
    
    if(name=="D*ExtraLite.NoKm")
    {
        return runDynamicExperiment<DStarExtraLiteNoKm_t, 
                    heuristic_search::SearchingDirection::Backward>;
    }
    
    if(name=="D*LiteOpt.")
    {
        return runDynamicExperiment<DStarLiteOptimized_t, 
                    heuristic_search::SearchingDirection::Backward>;
    }
    
    if(name=="D*Lite")
    {
        return runDynamicExperiment<DStarLite_t, 
                    heuristic_search::SearchingDirection::Backward>;
    }
    
    if(name=="MPGAA*")
    {
        return runDynamicExperiment<MPGAAStar_t, 
                    heuristic_search::SearchingDirection::Forward>;
    }
    
    return std::function<heuristic_search::loggers::Log(
        movingai_benchmark::benchmark::DynamicExperiment const& )>();
}

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
        > algorithm_selector)
{
    std::vector<std::function<heuristic_search::loggers::Log(
        movingai_benchmark::benchmark::DynamicExperiment const& )> > algorithms;
    
    for(auto name: names)
    {
        std::function<heuristic_search::loggers::Log(
        movingai_benchmark::benchmark::DynamicExperiment const& )> algorithm = 
                algorithm_selector(name);
        
        if(algorithm)
        {
            algorithms.push_back(algorithm);
        }
    }
    
    return algorithms;
}


}//namespace benchmark
}//namespace movingai_benchmark
