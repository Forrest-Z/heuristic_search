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


#include "search_domains/movingai_benchmark/benchmark/AStarRun.h"

#include <heuristic_search/AStar.h>
#include <heuristic_search/loggers/Logger.h>

#include <search_domains/movingai_benchmark/Domain.h>
#include <search_domains/movingai_benchmark/loader/MapLoader.h>

namespace movingai_benchmark{

namespace benchmark{

using namespace heuristic_search;
using namespace movingai_benchmark;

loggers::Log runAStar(loader::Experiment const& experiment, 
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
    
    int W;
    int H;
    unsigned char *map;
       
    if(!movingai_benchmark::loader::loadMap(experiment.GetMapName(),W,H,map))
    {
        return loggers::Log();
    }
        
    Domain domain(map, nullptr, W,H,0,1);
    
    AStar_t algorithm(domain, direction, preinitialized);
    
    Domain::State start(experiment.GetStartX(), experiment.GetStartY());
    Domain::State goal(experiment.GetGoalX(), experiment.GetGoalY());
    algorithm.search(start, goal);
    
    delete []map;
    
    return algorithm.log;
}

}//namespace benchmark
}//namespace movingai_benchmark
