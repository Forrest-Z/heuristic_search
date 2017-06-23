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

#include <search_domains/movingai_benchmark/loader/MapLoader.h>
#include <fstream>
#include <cassert>

namespace movingai_benchmark{
namespace loader{

bool loadMap(std::string const& filename, int &width, int &height, unsigned char*& map)
{
    std::ifstream map_file;
    
    map_file.open(filename.c_str());
    
    if(map_file.good())
    {    
        std::string type_tag;
        std::string width_tag;
        std::string height_tag;
        std::string map_tag;
        std::string type;
        
        map_file >> type_tag >> type;
        assert(type_tag=="type");
        
        map_file >> height_tag; 
        assert(height_tag=="height");
        map_file >> height;
        
        map_file >> width_tag; 
        assert(width_tag=="width");
        map_file >> width;
        
        map_file >> map_tag; 
        assert(map_tag=="map");
        
        assert(height>0 && width>0);
        
        map = new unsigned char[width*height];
        
        for(int i=0; i<width*height; i++)
        {
            char c;
            map_file >> c;
            map[i] = c;                    
        }

        return true;         
    }
    
    return false;
}


bool saveMap(std::string const& filename, int &width, int const&height, unsigned char* const&map)
{
    std::ofstream map_file;
    
    map_file.open(filename.c_str());
    
    if(map_file.good())
    {    
        std::string type_tag;
        std::string width_tag;
        std::string height_tag;
        std::string map_tag;
        std::string type;
        
        map_file << "type octile" << std::endl;
        map_file << "height " << height<< std::endl;
        map_file << "width " << width<< std::endl;
        map_file << "map" << std::endl;
                        
        for(int i=0; i<width*height; i++)
        {
            map_file << map[i];    
        }

        return true;         
    }
    
    return false;
}

}//namespace loader
}//namespace movingai_benchmark