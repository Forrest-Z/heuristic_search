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


#include "search_domains/movingai_benchmark/loader/ScenarioDirectoryLoader.h"

#include <iostream>
#include <iterator>
#include <fstream>

#include <boost/filesystem.hpp>


namespace movingai_benchmark{
namespace loader{

using namespace std;
using namespace boost::filesystem;

ScenarioDirectoryLoader::ScenarioDirectoryLoader(std::string const& scenario_directory_path,
            std::string const& maps_directory_path, bool recursive)
{
    scenario_directory_path_ = scenario_directory_path;
    maps_directory_path_ = maps_directory_path;

    load(scenario_directory_path_, recursive);

    cout << experiments_.size() << " experiments loaded.\n";
}

ScenarioDirectoryLoader::~ScenarioDirectoryLoader()
{

}

void ScenarioDirectoryLoader::load(std::string const& directory_path, bool recursive)
{
    path p (directory_path);

    try
    {
        if (exists(p))
        {
            if (is_regular_file(p))
            {
                loadFile(p.string());
            }
            else if (is_directory(p))
            {
                directory_iterator it = directory_iterator(p);

                while(it!=directory_iterator())
                {
                    path sp = it->path();

                    if (is_regular_file(sp))
                    {
                        loadFile(sp.string());
                    }
                    else if (is_directory(sp) && recursive)
                    {
                        load(sp.string(),recursive);
                    }

                    it++;
                }

            }
            else
            {
                cout << p << " exists, but is neither a regular file nor a directory\n";
            }
        }
        else
        {
            cout << p << " does not exist\n";
        }
    }

    catch (const filesystem_error& ex)
    {
      cout << ex.what() << '\n';
    }
}

void ScenarioDirectoryLoader::loadFile(std::string const& file_path)
{
    path p (file_path);

    if(p.extension().string()!=".scen")
    {
        cout <<"Ignoring file with unknown extension: " << file_path << "\n";
        return;
    }

    cout <<"Loading file: " << file_path << "\n";

    ScenarioLoader loader(file_path.c_str());

    std::vector<Experiment>::iterator it = loader.GetExperiments().begin();
    std::vector<Experiment>::iterator it_e = loader.GetExperiments().end();

    for(;it!=it_e;it++)
    {
        Experiment experiment(
        it->GetStartX(), it->GetStartY(), it->GetGoalX(), it->GetGoalY(),
                it->GetXScale(), it->GetYScale(), it->GetBucket(), it->GetDistance(),
                maps_directory_path_ + "/" + it->GetMapName());

        path p(experiment.GetMapName());
        assert(exists(p));

        experiments_.push_back(experiment);
    }

}

void ScenarioDirectoryLoader::saveToFile(std::string const& file_path,
        std::vector<Experiment> const& experiments)
{
    path p (file_path);

    if(p.extension().string()!=".scen")
    {
        cout <<"Ignoring file with unknown extension: " << file_path << "\n";
        return;
    }

    if(!boost::filesystem::exists(p.parent_path()))
    {
        if(!boost::filesystem::create_directories(p.parent_path()))
        {
            std::cerr << "Couldn't create directory " << p.parent_path() << std::endl;

            return;
        }
    }

    std::ofstream ofile(file_path.c_str());
    if(!ofile.good())
    {
        std::cerr << "Couldn't open file " << file_path << std::endl;

        return;
    }


    float ver = 1.0;
    ofile<<"version "<<ver<<std::endl;


    for (unsigned int x = 0; x < experiments.size(); x++)
    {
        std::string map_name = experiments[x].GetMapName();
        map_name = map_name.substr(maps_directory_path_.size()+1,
                map_name.size() - maps_directory_path_.size()-1);

        ofile<<experiments[x].GetBucket()<<"\t"<<map_name<<"\t"<<experiments[x].GetXScale()<<"\t";
        ofile<<experiments[x].GetYScale()<<"\t"<<experiments[x].GetStartX()<<"\t"<<experiments[x].GetStartY()<<"\t";
        ofile<<experiments[x].GetGoalX()<<"\t"<<experiments[x].GetGoalY()<<"\t"<<experiments[x].GetDistance()<<std::endl;
    }

    std::cout << "Experiments written to \n" << file_path << std::endl << std::endl;
}

}//namespace loader
}//namespace movingai_benchmark
