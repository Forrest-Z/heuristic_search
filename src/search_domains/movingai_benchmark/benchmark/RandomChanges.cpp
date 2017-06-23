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
#include <cstring>
#include <exception>
#include <iostream>

#include <boost/lexical_cast.hpp>

#include "search_domains/movingai_benchmark/benchmark/RandomChanges.h"
#include "search_domains/movingai_benchmark/Domain.h"
#include "search_domains/movingai_benchmark/loader/MapLoader.h"
#include "search_domains/movingai_benchmark/loader/ScenarioDirectoryLoader.h"

namespace movingai_benchmark{
namespace benchmark{

//const char UNKNOWN_TERRAIN = '.';
//const char PASSABLE_TERRAIN = '.';
//const char PASSABLE_TERRAIN_G = 'G';
//const char OUT_OF_BOUNDS = '@';
//const char OUT_OF_BOUNDS_O = 'O';
//const char TREES = 'T';
//const char SWAMP = 'S';
//const char WATER = 'W';
bool isObstacle(unsigned char map_value)
{
    switch(map_value)
    {
        case movingai_benchmark::PASSABLE_TERRAIN: return false;
        case movingai_benchmark::PASSABLE_TERRAIN_G: return false;
        case movingai_benchmark::OUT_OF_BOUNDS: return true;
        case movingai_benchmark::OUT_OF_BOUNDS_O: return true;
        case movingai_benchmark::TREES: return true;
        case movingai_benchmark::SWAMP: return true;
        case movingai_benchmark::WATER: return true;
    }
    
    return true;
}

void drawBlock( unsigned char * map, 
        int W, int H, int x, int y,
        int block_size, unsigned char map_value)
{    
    for(int i=std::max(0,x-block_size); i<=std::min(W-1,x+block_size); ++i)
    {
        for(int j=std::max(0,y-block_size); j<=std::min(H-1,y+block_size); ++j)
        {
            map[j*W + i] = map_value;
        }
    }
    
}

unsigned char * createMapWithRandomChanges( unsigned char const* map, 
        int W, int H, double change_rate)
{
    int cells_number = W*H;
    int unblock = change_rate/2.0 * cells_number;
    int block = change_rate/2.0 * cells_number;
    
//    int block_size = W/2;

    unsigned char *changed_map = new unsigned char[W*H];
    std::memcpy(changed_map, map, W*H*sizeof(unsigned char));
    
    int i=0;
    while(i<unblock)
    {
        int id = std::rand()%cells_number;

        if(isObstacle(map[id]))
        {
//            int bs = std::rand()%(block_size+1);
            int bs = 0;
            drawBlock(changed_map, W, H, id%W, id/W, bs,
                    movingai_benchmark::PASSABLE_TERRAIN);
            i+=(bs+1)*(bs+1);
        }
    }

    i=0;
    while(i<block)
    {
        int id = std::rand()%cells_number;

        if(!isObstacle(map[id]))
        {
//            int bs = std::rand()%(block_size+1);
            int bs = 0;
            drawBlock(changed_map, W, H, id%W, id/W, bs,
                    movingai_benchmark::TREES);
            i+=(bs+1)*(bs+1);
        }
    }
    
    return changed_map;
}

DynamicExperiment initializeExperimentWithRandomChanges(
    loader::Experiment const& experiment, int observation_range, double change_rate)
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
    unknown_terrain_experiment.observation_range = observation_range;
    unknown_terrain_experiment.W = W;
    unknown_terrain_experiment.H = H;
    unknown_terrain_experiment.true_map = map;
    unknown_terrain_experiment.known_map = createMapWithRandomChanges(
            map, W, H, change_rate);
    
    return unknown_terrain_experiment;
}


void runBenchmarkWithRandomChanges(
    std::string hog_benchmarks_dir,
    std::string problems_name,
    std::string test_name,
    bool preinitialized, 
    int observation_range,
    double change_rate,
    std::vector<std::function<heuristic_search::loggers::Log(DynamicExperiment const&)> > algorithms,
    int problems_count,
    std::function<int(std::vector<loader::Experiment> const&)> experiment_id_selector,
    std::function<heuristic_search::loggers::Log(DynamicExperiment const&)> problem_validator
    )
{
    loader::ScenarioDirectoryLoader scenarios(
            hog_benchmarks_dir+"/problems/" + problems_name,
            hog_benchmarks_dir);
    
    
    if(scenarios.experiments().empty())
    {
        return;
    }
                 
    auto result = heuristic_search::benchmark::runBenchmark<loader::Experiment, DynamicExperiment>(
            scenarios.experiments(), 
            problem_validator, algorithms,
            problems_count, 
            experiment_id_selector, 
            std::bind(initializeExperimentWithRandomChanges, std::placeholders::_1, observation_range, change_rate)); 
    
    auto logs = result.first;
    auto experiments = result.second;
    
    heuristic_search::benchmark::printSummary(logs, 4, 1);
    
    std::string preinit = (preinitialized)?"preinit":"uninit";    
    std::string observ = boost::lexical_cast<std::string>(observation_range);    
    std::string cr_id = boost::lexical_cast<std::string>(int(change_rate*100));    
    if(cr_id.size()==1)
        cr_id = "0" + cr_id;
    
    heuristic_search::benchmark::writeResults(hog_benchmarks_dir, 
            test_name+"/"+problems_name + "/" + problems_name + "_cr"+cr_id + "_" + preinit + "_obs" + observ,
            experiments, logs, 4, 1);
}

}//namespace benchmark
}//namespace movingai_benchmark
