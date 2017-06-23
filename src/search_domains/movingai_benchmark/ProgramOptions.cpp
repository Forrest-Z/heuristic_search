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


#include "search_domains/movingai_benchmark/ProgramOptions.h"

namespace movingai_benchmark{

namespace po = boost::program_options;

boost::program_options::options_description generalOptions()
{
    po::options_description general_ops("Available options");
    general_ops.add_options()
        ("help", "this help message")
    ;

    return general_ops;
}

boost::program_options::options_description algorithmsOptions()
{
    po::options_description general_ops;
    general_ops.add_options()
        ("algorithms", po::value<std::vector<std::string> >()->multitoken()->required(),
            "list of algorithms to compare. \nAvailable algorithms: \n"
            "    D*ExtraLite, D*ExtraLite.NoKm,\n"
            "    D*LiteOpt., D*Lite, MPGAA*")
    ;

    return general_ops;
}

boost::program_options::options_description dynamicExperimentOptions()
{
    po::options_description general_ops("Dynamic experiment params");
    general_ops.add_options()
        ("observation-range", po::value<int>()->default_value(10),
            "observation range")
        ("preinitialized", po::value<bool>()->default_value(false)->zero_tokens(),
            "the search-space will be initialized in advance,"
            " otherwise search-space nodes are allocated bit-by-bit")
        ("undirected",  po::value<bool>()->default_value(false)->zero_tokens(),
            "the same-pointer list will be used for predecessors and successors")
    ;

    return general_ops;
}

boost::program_options::options_description singleProblemOptions()
{
    po::options_description single_problem_ops("Problem definition");
    single_problem_ops.add_options()
        ("map-file", po::value<std::string>()->required(), "path to the map file")
        ("start.x", po::value<int>()->required(), "start.x")
        ("start.y", po::value<int>()->required(), "start.y")
        ("goal.x", po::value<int>()->required(), "goal.x")
        ("goal.y", po::value<int>()->required(), "goal.y")
    ;

    return single_problem_ops;
}

boost::program_options::options_description benchmarkOptions()
{
    po::options_description problems_ops("Multiple problems options");
    problems_ops.add_options()
        ("benchmark-dir", po::value<std::string>()->default_value("."),
            "path to the main directory containing subdirectories called 'maps' and 'problems'")
        ("problems-path", po::value<std::string>()->required(),
            "path to the directory containing problem files or to the single problem file (relative to <benchmark-dir>)")
        ("problems-number", po::value<int>()->required(),
            "how many problems to solve (to be used with <problems-path>)")
        ("problems-random", po::value<bool>()->default_value(true)->zero_tokens(),
            "the problem(s) will be selected randomly. If not defined,"
            " subsequent problems will be selected starting from the beginning (to be used with <problems-path>)")
        ("test-name", po::value<std::string>()->default_value("movingai_benchmark"),
            "name of the test - used in result files naming")
//TODO
//        ("write-logs", po::value<bool>()->default_value(true)->zero_tokens(),
//            "if defined, the logs will be written to files (optional)")
//        ("log-filename", po::value<std::string>(),
//            "custom name for log files (optional)")
    ;

    return problems_ops;
}

boost::program_options::options_description visualizationOptions()
{
    po::options_description visualization_ops("Visualization");
    visualization_ops.add_options()
        ("visualization", po::value<bool>()->default_value(false)->zero_tokens(), "visualization will be shown")
        ("vis-scale", po::value<double>()->default_value(1.0), "visualization will be scaled by this value")
        ("vis-delay", po::value<int>()->default_value(5), "visualization delay in ms. 0 delay runs step-by-step mode.")
        ("vis-hide-text", po::value<bool>()->default_value(false)->zero_tokens(), "hides text in the visualization.")
    ;

    return visualization_ops;
}

boost::program_options::options_description configOptions()
{
    po::options_description config_ops("Config");
    config_ops.add_options()
        ("config-file", po::value<std::string>(), "all parameters can be loaded from a config file")
    ;

    return config_ops;
}


}
