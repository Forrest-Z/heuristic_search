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


#ifndef MOVINGAI_BENCHMARK_BENCHMARK_FREESPACE_H
#define MOVINGAI_BENCHMARK_BENCHMARK_FREESPACE_H

#include "search_domains/movingai_benchmark/benchmark/DynamicExperiment.h"

#include "heuristic_search/benchmark/Benchmark.h"
#include "search_domains/movingai_benchmark/loader/ScenarioLoader.h"
#include "search_domains/movingai_benchmark/benchmark/AStarRun.h"


namespace movingai_benchmark{

namespace benchmark{

DynamicExperiment initializeFreespaceExperiment(loader::Experiment const& experiment,
        DynamicExperimentParams const& params);


void runFreespaceBenchmark(
    std::string hog_benchmarks_dir,
    std::string problems_name,
    std::string test_name,
    bool preinitialized,
    bool undirected,
    int observation_range,
    std::vector<std::function<heuristic_search::loggers::Log(DynamicExperiment const&)> > algorithms,
    int problems_count,
    std::function<int(std::vector<loader::Experiment> const&)> experiment_id_selector =
        std::function<int(std::vector<loader::Experiment> const&)>(),
    std::function<heuristic_search::loggers::Log(DynamicExperiment const&)> problem_validator =
        std::bind(movingai_benchmark::benchmark::runAStarForDynamicExperiment, std::placeholders::_1,
            heuristic_search::SearchingDirection::Forward, false)
    );

}//namespace benchmark

}//namespace movingai_benchmark

#endif /* MOVINGAI_BENCHMARK_BENCHMARK_FREESPACE_H */
