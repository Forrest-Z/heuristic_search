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


#include "heuristic_search/benchmark/Benchmark.h"

#include <cmath>
#include <iomanip>

namespace heuristic_search{

namespace benchmark{

std::vector<loggers::Log> mean(std::vector<std::vector<loggers::Log> > logs, int min_problems_number)
{
    std::vector<loggers::Log> mean_log;
    std::vector<loggers::Log> sum_log;

    if(logs.empty() || static_cast<int>(logs.size()) < min_problems_number)
    {
        return mean_log;
    }

    mean_log.resize(logs.front().size());
    sum_log.resize(logs.front().size());

    for(int i=0; i<static_cast<int>(logs.front().size()); ++i)
    {
        sum_log[i].algorithm_name = logs.front()[i].algorithm_name;
    }

    int valid_problems = 0;

    for(auto problem_logs : logs)
    {
        bool all_succeed = true;

        for(int i=0; i<int(problem_logs.size()); ++i)
        {
            if(!problem_logs[i].success)
            {
                all_succeed = false;
                break;
            }
        }

        if(all_succeed)
        {
            valid_problems++;

            for(int i=0; i<int(problem_logs.size()); ++i)
            {
                if(problem_logs[i].success)
                {
                    sum_log[i] += problem_logs[i];
                }
            }
        }
    }

    if(valid_problems < min_problems_number)
    {
        return std::vector<loggers::Log>();
    }


    for(int i=0; i<int(sum_log.size()); ++i)
    {
        if(sum_log[i].success)
        {
            mean_log[i] = sum_log[i];
            mean_log[i] /= sum_log[i].success;
            mean_log[i].success = sum_log[i].success;
        }
    }

    return mean_log;
}

std::vector<std::vector<std::vector<loggers::Log> > > putInBuckets(
    std::vector<std::vector<loggers::Log> > logs,
        double bucket_cost_range)
{
    std::vector<std::vector<std::vector<loggers::Log> > > buckets;

    int bucket_offset = 0;

    for(auto problem_logs : logs)
    {
        if(problem_logs.empty())
        {
            continue;
        }
        
        int bucket = bucket_offset + std::floor(problem_logs.front().path_cost / bucket_cost_range);

        if(static_cast<int>(buckets.size())<=bucket)
        {
            buckets.resize(bucket+1);
        }
        else if(bucket<0)
        {
            bucket_offset += -bucket;
            buckets.insert(buckets.begin(), -bucket, std::vector<std::vector<loggers::Log> >());
            bucket = 0;
        }

        buckets[bucket].push_back(problem_logs);
    }

    return buckets;
}

std::vector<std::vector<loggers::Log> > meanBuckets(
        std::vector<std::vector<loggers::Log> > logs,
        double bucket_cost_range, int min_problems_number)
{
    auto buckets = putInBuckets(logs, bucket_cost_range);

    std::vector<std::vector<loggers::Log> > mean_buckets;

    for(auto bucket : buckets)
    {
        mean_buckets.push_back(mean(bucket, min_problems_number));
    }

    return mean_buckets;
}

void writeSummary(std::ostream &ostream, std::vector<std::vector<loggers::Log> > logs,
        double bucket_cost_range, int min_problems_number)
{
    ostream << std::endl << "---Summary---" << std::endl;
    ostream << "Total problem solved: " << logs.size() << std::endl;

    auto log_buckets = putInBuckets(logs, bucket_cost_range);

    ostream << std::endl << "---average in buckets with at least "
            << min_problems_number << " problems---" << std::endl;
    ostream << std::endl;

    int bucket_id = 0;
    for(auto log_bucket : log_buckets)
    {
        auto mean_bucket = mean(log_bucket, min_problems_number);

        if(mean_bucket.empty())
        {
            bucket_id++;
            continue;
        }


        ostream
                << "bucket: " << bucket_id++
                << "\t problems: " << log_bucket.size()
                << std::endl;

        loggers::Log::writeHeader(ostream);
        ostream << std::endl;

        for(auto log : mean_bucket)
        {
            ostream << log << std::endl;
        }

        ostream << std::endl;
    }

    auto mean_logs = benchmark::mean(logs);
    ostream << std::endl << "---average in total---" << std::endl;
    ostream << std::endl;

    loggers::Log::writeHeader(ostream);
    ostream << std::endl;

    for(auto log : mean_logs)
    {
        ostream << log << std::endl;
    }

    ostream << std::endl;
}

void writeSummaryToFile(std::string filename,
        std::vector<std::vector<loggers::Log> > logs,
        double bucket_cost_range, int min_problems_number)
{

    boost::filesystem::path path(filename);

    if(!boost::filesystem::exists(path.parent_path()))
    {
        if(!boost::filesystem::create_directories(path.parent_path()))
        {
            std::cerr << "Couldn't create directory " << path.parent_path() << std::endl;

            return;
        }
    }

    std::ofstream file(filename.c_str());
    if(!file.good())
    {
        std::cerr << "Couldn't open file " << filename << std::endl;

        return;
    }

    writeSummary(file, logs, bucket_cost_range, min_problems_number);

    std::cout << "Summary written to \n" << filename << std::endl << std::endl;
}

void printSummary(std::vector<std::vector<loggers::Log> > logs,
        double bucket_cost_range, int min_problems_number)
{
    writeSummary(std::cout, logs, bucket_cost_range, min_problems_number);
}

void writeBucketLogsToFile(std::string filename,
        std::vector<std::vector<loggers::Log> > buckets)
{
    boost::filesystem::path path(filename);

    if(!boost::filesystem::exists(path.parent_path()))
    {
        if(!boost::filesystem::create_directories(path.parent_path()))
        {
            std::cerr << "Couldn't create directory " << path.parent_path() << std::endl;

            return;
        }
    }

    std::ofstream file(filename.c_str());
    if(!file.good())
    {
        std::cerr << "Couldn't open file " << filename << std::endl;

        return;
    }

    if(buckets.empty())
    {
        return;
    }

    bool header_written = false;

    for(int i=0; i<static_cast<int>(buckets.size()); ++i)
    {
        if(buckets[i].empty())
        {
            continue;
        }

        if(!header_written)
        {
            header_written = true;

            file << std::setw(7) << std::right << "bucket";
            for(int j=0; j<static_cast<int>(buckets[i].size()); ++j)
            {
                loggers::Log::writeHeader(file);
            }
            file << std::endl;
        }

        file << std::setw(7) << std::right << i;

        for(auto log : buckets[i])
        {
            file << log;
        }

        file << std::endl;
    }

    std::cout << "Bucket logs written to \n" << filename << std::endl << std::endl;

}


}//namespace benchmark

}//namespace heuristic_search
