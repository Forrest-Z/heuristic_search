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


#include "search_domains/movingai_benchmark/benchmark/Benchmark.h"
#include "search_domains/movingai_benchmark/loader/ScenarioDirectoryLoader.h"

namespace movingai_benchmark{

namespace benchmark{

void runBenchmark(
    std::string hog_benchmarks_dir,
    std::string problems_name,
    std::string test_name,
    std::vector<std::function<heuristic_search::loggers::Log(loader::Experiment const&)> > algorithms,
    int problems_count,
    std::function<int(std::vector<loader::Experiment> const&)> experiment_id_selector,
    std::function<heuristic_search::loggers::Log(loader::Experiment const&)> problem_validator)
{
    loader::ScenarioDirectoryLoader scenarios(
            hog_benchmarks_dir+"/problems/" + problems_name,
            hog_benchmarks_dir);
                 
    auto result = heuristic_search::benchmark::runBenchmark(scenarios.experiments(), 
            problem_validator, algorithms,
            problems_count, experiment_id_selector); 
    
    auto logs = result.first;
    auto experiments = result.second;
    
    heuristic_search::benchmark::printSummary(logs, 4, 1);
        
    heuristic_search::benchmark::writeResults(hog_benchmarks_dir, 
            test_name+"_"+problems_name,
            experiments, logs, 4, 1);
}

}//namespace benchmark
}//namespace movingai_benchmark
