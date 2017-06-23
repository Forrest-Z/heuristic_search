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


#ifndef STDHEAP_H
#define STDHEAP_H

#include <vector>
#include <algorithm>
#include <cassert>



namespace heuristic_search{


template<typename Key_t>
class StdHeapElement
{
public:

    StdHeapElement(Key_t key) : key(key)
    {
        open = false;
        closed = false;
    }

    virtual ~StdHeapElement()
    {

    }

    Key_t key;

    bool open;
    bool closed;
};

template<typename Key_t>
class StdHeap
{
public:
    typedef StdHeapElement<Key_t> StdHeapElement_t;

    typedef std::pair<Key_t, StdHeapElement_t*> heap_pair_t;
    typedef std::vector<heap_pair_t> heap_container_t;

    StdHeap(Key_t key_max) : key_max_(key_max)
    {

    }

    virtual ~StdHeap()
    {
        clear();
    }

    StdHeapElement_t *topOpen()
    {
        updateOpen();

        if(empty())
        {
            return nullptr;
        }

        return top().second;
    }

    Key_t topKey()
    {
        updateOpen();

        if(empty())
        {
            return key_max_;
        }

        return top().first;
    }

    void popOpen()
    {
        updateOpen();

        if(!empty())
        {
            StdHeapElement_t *top_node = topOpen();
            top_node->open = false;
            top_node->closed = true;

            pop();
        }
    }

    void pushOpen(StdHeapElement_t *v, Key_t key)
    {            
        if(v->open && !v->closed && v->key == key)
            return;

        v->key = key;
        v->open = true;
        v->closed = false;

        push(heap_pair_t(key,v));
    }

    void removeOpen(StdHeapElement_t *v)
    {
        v->open = false;
        v->closed = true;
    }

    bool empty()
    {
        updateOpen();

        return heap_.empty();
    }

    void clear()
    {
        for(auto &heap_pair : heap_)
        {
            if(heap_pair.second)
            {
                heap_pair.second->open = false;
            }
        }

        heap_.clear();
    }

    heap_container_t heap()
    {
        return heap_;
    }

    void makeHeap(heap_container_t & heap)
    {
        heap_.swap(heap);

        std::make_heap(heap_.begin(), heap_.end(), Compare());

    }

protected:
    heap_container_t heap_;
    Key_t key_max_;


    void updateOpen()
    {
        if(heap_.empty())
            return;

        heap_pair_t top_pair = top();

        while(true)
        {
            if( (top_pair.first>top_pair.second->key
                    || top_pair.first == top_pair.second->key)
                && !top_pair.second->closed
                && top_pair.second->open)
            {
                break;
            }

            pop();

            if(heap_.empty())
            {
                break;
            }

            top_pair = top();
        }
    }


    struct Compare
    {
        bool operator()(heap_pair_t const& a, heap_pair_t const& b)
        {
            return a.first > b.first;
        }
    };

    heap_pair_t top()
    {
        assert(!heap_.empty());
        return heap_.front();
    }

    void pop()
    {
        assert(!heap_.empty());

        std::pop_heap (heap_.begin(),heap_.end(),Compare());
        heap_.pop_back();
    }

    void push(heap_pair_t  const& v)
    {
        heap_.push_back(v);
        std::push_heap (heap_.begin(),heap_.end(),Compare());
    }


};

}//namespace heuristic_search

#endif /* STDHEAP_H */
