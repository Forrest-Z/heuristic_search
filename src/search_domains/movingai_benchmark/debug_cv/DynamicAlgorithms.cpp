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


#include "search_domains/movingai_benchmark/debug_cv/DynamicAlgorithms.h"


#include <heuristic_search/AStar.h>
#include <heuristic_search/DStarExtraLite.h>
#include <heuristic_search/DStarExtraLiteNoKm.h>
#include <heuristic_search/DStarLiteOptimized.h>
#include <heuristic_search/DStarLite.h>
#include <heuristic_search/DStarMain.h>
#include "heuristic_search/MPGAAStar.h"
#include "search_domains/movingai_benchmark/ProgramOptions.h"
#include <heuristic_search/IncrementalSearch.h>
#include <heuristic_search/loggers/Logger.h>
#include <heuristic_search/loggers/DStarLogger.h>
#include <heuristic_search/loggers/MPGAALogger.h>

#include <search_domains/movingai_benchmark/debug_cv/debug_cv.h>
#include <search_domains/movingai_benchmark/debug_cv/Search.h>
#include <search_domains/movingai_benchmark/debug_cv/DStarMain.h>

namespace movingai_benchmark{
namespace debug_cv{

heuristic_search::loggers::Log runAStarForDynamicExperiment(
        movingai_benchmark::benchmark::DynamicExperiment const& experiment, 
        heuristic_search::SearchingDirection direction, 
        std::string const& name, VisualizationConfig vis_config)
{
    typedef heuristic_search::SearchAlgorithmBegin<  
            heuristic_search::loggers::Logger<
            heuristic_search::SearchLoop<
            heuristic_search::AStar<
            heuristic_search::HeuristicSearch<
            heuristic_search::StdOpenList<
            heuristic_search::StdSearchSpace<
        heuristic_search::SearchAlgorithmEnd<Domain> > > > > > > >::Algorithm_t AStar_t;
    
            
    Domain domain(experiment.true_map, nullptr, experiment.W,experiment.H,0,1);
    
    AStar_t algorithm(domain, direction, experiment.preinitialized, 
            experiment.undirected);
        
    Domain::State start(experiment.experiment.GetStartX(), experiment.experiment.GetStartY());
    Domain::State goal(experiment.experiment.GetGoalX(), experiment.experiment.GetGoalY());
    algorithm.search(start, goal);
    
    vis_config.delay = 0;
    
    std::vector<std::string> text;
    
    if(!vis_config.hide_text)
    {    
        text.push_back(name);
        text.push_back("press ESC to quit");

        if(vis_config.delay==0)
        {
            text.push_back("or any other key to continue...");
        }
    }
    
    movingai_benchmark::debug_cv::drawAlgorithm(name.c_str(), 
            domain.map_, domain.W, domain.H, 
            start, 
            goal, 
            algorithm, 
            vis_config.scale,
            text);
    
    if(cv::waitKey(vis_config.delay)==27)
    {
        cv::destroyWindow(name);
        return heuristic_search::loggers::Log();
    }
            
    cv::destroyWindow(name);
    return algorithm.log;
}

typedef heuristic_search::SearchAlgorithmBegin<  
        movingai_benchmark::debug_cv::DStarMain<  
        movingai_benchmark::debug_cv::Search<   
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
    heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > > > > > > >::Algorithm_t DStarExraLite_t;
   

   
typedef heuristic_search::SearchAlgorithmBegin< 
        movingai_benchmark::debug_cv::DStarMain<  
        movingai_benchmark::debug_cv::Search<   
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
    heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > > > > > > > >::Algorithm_t DStarExtraLiteNoKm_t;
    
        

typedef heuristic_search::SearchAlgorithmBegin<
        movingai_benchmark::debug_cv::DStarMain<  
        movingai_benchmark::debug_cv::Search<    
        heuristic_search::loggers::DStarLogger<      
        heuristic_search::DStarMain<
        heuristic_search::SearchLoop<
        heuristic_search::DStarLiteOptimized<
        heuristic_search::HeuristicSearch<
        heuristic_search::StdOpenList<
        heuristic_search::StdSearchSpace<
    heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > > > >::Algorithm_t DStarLiteOptimized_t;
    


typedef heuristic_search::SearchAlgorithmBegin< 
        movingai_benchmark::debug_cv::DStarMain<  
        movingai_benchmark::debug_cv::Search<   
        heuristic_search::loggers::DStarLogger<     
        heuristic_search::DStarMain<
        heuristic_search::SearchLoop<
        heuristic_search::DStarLite<
        heuristic_search::HeuristicSearch<
        heuristic_search::StdOpenList<
        heuristic_search::StdSearchSpace<
    heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > > > >::Algorithm_t DStarLite_t;
 


typedef heuristic_search::SearchAlgorithmBegin<  
        movingai_benchmark::debug_cv::DStarMain<    
        movingai_benchmark::debug_cv::Search<
        heuristic_search::loggers::MPGAALogger<        
        heuristic_search::DStarMain<
        heuristic_search::MPGAAStar<
        heuristic_search::HeuristicSearch<
        heuristic_search::StdOpenList<
        heuristic_search::StdSearchSpace<
    heuristic_search::SearchAlgorithmEnd<movingai_benchmark::Domain> > > > > > > > > >::Algorithm_t MPGAAStar_t;


template<typename Algorithm_t, heuristic_search::SearchingDirection search_direction>
heuristic_search::loggers::Log runDynamicExperiment(
    movingai_benchmark::benchmark::DynamicExperiment const& experiment, 
        std::string const& name, VisualizationConfig vis_config)
{        
    
    movingai_benchmark::Domain domain(experiment.known_map, experiment.true_map, 
            experiment.W, experiment.H, experiment.observation_range,1);
    
    Algorithm_t algorithm(domain, search_direction, experiment.preinitialized, 
            experiment.undirected);
              
    algorithm.cv_window_name = name;
    algorithm.cv_scale = vis_config.scale;
    algorithm.cv_window_delay = vis_config.delay;
    algorithm.cv_hide_text = vis_config.hide_text;
    
    algorithm.main(
        movingai_benchmark::Domain::State(experiment.experiment.GetStartX(),
                                        experiment.experiment.GetStartY()),
        movingai_benchmark::Domain::State(experiment.experiment.GetGoalX(), 
                                        experiment.experiment.GetGoalY())
    );
    
        
    heuristic_search::loggers::Log log = algorithm.result();
    
    cv::destroyWindow(name);
        
    return log;
}


std::function<heuristic_search::loggers::Log(
        movingai_benchmark::benchmark::DynamicExperiment const& )> getDynamicAlgorithm(
            std::string const& name, VisualizationConfig vis_config)
{
    if(name=="D*ExtraLite")
    {
        return std::bind(
                    runDynamicExperiment<DStarExraLite_t, 
                        heuristic_search::SearchingDirection::Backward>,
                    std::placeholders::_1,
                    name, 
                    vis_config
                );
    }
    
    if(name=="D*ExtraLite.NoKm")
    {
        return std::bind(
                    runDynamicExperiment<DStarExtraLiteNoKm_t, 
                        heuristic_search::SearchingDirection::Backward>,
                    std::placeholders::_1,
                    name, 
                    vis_config
                );
    }
    
    if(name=="D*LiteOpt.")
    {
        return std::bind(
                    runDynamicExperiment<DStarLiteOptimized_t, 
                        heuristic_search::SearchingDirection::Backward>,
                    std::placeholders::_1,
                    name, 
                    vis_config
                );
    }
    
    if(name=="D*Lite")
    {
        return std::bind(
                    runDynamicExperiment<DStarLite_t, 
                        heuristic_search::SearchingDirection::Backward>,
                    std::placeholders::_1,
                    name, 
                    vis_config
                );
    }
    
    if(name=="MPGAA*")
    {
        return std::bind(
                    runDynamicExperiment<MPGAAStar_t, 
                        heuristic_search::SearchingDirection::Forward>,
                    std::placeholders::_1,
                    name, 
                    vis_config
                );
    }
    
    return std::function<heuristic_search::loggers::Log(
        movingai_benchmark::benchmark::DynamicExperiment const& )>();
}

}//namespace debug_cv
}//namespace movingai_benchmark

