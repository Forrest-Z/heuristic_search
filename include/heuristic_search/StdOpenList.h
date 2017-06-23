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


#ifndef HEURISTIC_SEARCH_STDOPENLIST_H
#define HEURISTIC_SEARCH_STDOPENLIST_H

#include "heuristic_search/StdHeap.h"

namespace heuristic_search{

template<typename B>
class StdOpenList : public B
{
public:

    template<typename final_t>
    class Node :    public B::template Node<final_t>,
                    public StdHeapElement<typename final_t::Key_t>
    {
    public:
        typedef typename final_t::Key_t Key_t;
        typedef StdHeapElement<typename final_t::Key_t> StdHeapElement_t;

        Node() : StdHeapElement_t(final_t::keyMax())
        {

        }

        virtual ~Node()
        {

        }

        void resetSearchData()
        {
            StdHeapElement_t::key = final_t::keyMax();
            StdHeapElement_t::open = false;
            StdHeapElement_t::closed = false;
        }
    };

    template<typename final_t>
    class OpenList :    public B::template OpenList<final_t>,
                        public StdHeap<typename final_t::Key_t>
    {
    public:

        typedef StdHeap<typename final_t::Key_t> StdHeap_t;

        OpenList() : StdHeap_t(final_t::keyMax())
        {

        }

        typename final_t::Node_t *topOpen()
        {
            return static_cast<typename final_t::Node_t *>(StdHeap_t::topOpen());
        }        

    };

};

}//namespace heuristic_search
#endif /* HEURISTIC_SEARCH_STDOPENLIST_H */
