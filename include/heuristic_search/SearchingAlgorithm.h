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


#ifndef HEURISTIC_SEARCH_SEARCHINGALGORITHM_H
#define HEURISTIC_SEARCH_SEARCHINGALGORITHM_H

namespace heuristic_search{


template<typename Domain>
class SearchAlgorithmEnd
{
public:    
    typedef Domain Domain_t;
    
    template<typename final_t> 
    class Node 
    {
    public:
        virtual ~Node()
        {
            
        }
    };
    
        
    
    template<typename final_t> 
    class OpenList
    {
    public:
        virtual ~OpenList()
        {
            
        }
        
    };
    
    template<typename final_t> 
    class Algorithm
    {
    public:
        virtual ~Algorithm()
        {
            
        }
        
    };
    
    template<typename final_t>
    class SearchSpace
    {
    public:
        virtual ~SearchSpace()
        {
            
        }
        
    };
};


template<class LAYERS>
class SearchAlgorithmBegin : public LAYERS
{
public:
    typedef SearchAlgorithmBegin<LAYERS> final_t;
            
    typedef typename final_t::template Node<final_t> Node_t;
    
    typedef typename final_t::template SearchSpace<final_t> SearchSpace_t;
    
    typedef typename final_t::template OpenList<final_t > OpenList_t;
    
    typedef typename final_t::template Algorithm<final_t> Algorithm_t;    
    
};

}//namespace heuristic_search

#endif /* HEURISTIC_SEARCH_SEARCHINGALGORITHM_H */

