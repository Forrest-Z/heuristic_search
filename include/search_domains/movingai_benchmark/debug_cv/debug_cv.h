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


#ifndef MOVINGAI_BENCHMARK_DEBUG_OPENCV_H
#define MOVINGAI_BENCHMARK_DEBUG_OPENCV_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "search_domains/movingai_benchmark/Domain.h"


namespace movingai_benchmark{
namespace debug_cv{


void drawPoint(int x, int y, cv::Mat &img, int channel, double scale=1.0);
void drawCircle(int x, int y, cv::Mat &img, double scale=1.0,int size=3);
void drawLine(int from_x, int from_y, int to_x, int to_y, cv::Mat &img, int channel, double scale=1.0);
void drawCross(int x, int y, int W, int H, cv::Mat &img, double scale=1.0,int size=3);

void drawMap(unsigned char *map, int W, int H, cv::Mat &img, double scale=1.0, int color=7, unsigned char intens=0);
void drawFancyMap(unsigned char *map, int W, int H, cv::Mat &img, double scale=1.0);
void drawState(Domain::State const& state, int W, int H, cv::Mat &img, int channel, double scale=1.0);

void drawCircle(Domain::State const& state, int W, int H, cv::Mat &img, double scale=1.0,int size=3);
void drawLine(Domain::State const& from, Domain::State const& to, int W, int H, cv::Mat &img, int channel, double scale=1.0);
void drawCross(Domain::State const& state, int W, int H, cv::Mat &img, double scale=1.0,int size=3);
void drawText(std::vector<std::string> text, int W, int H, cv::Mat &img, double scale=1.0);

template<typename State>
void drawStates(std::vector<State> const& states, int W, int H, cv::Mat &img, double scale=1.0)
{
    if(img.empty())
    {
        img = cv::Mat::zeros(cv::Size(W*scale,H*scale), CV_8UC3);
    }

    for(auto const& state : states)
    {
        drawState(state,W,H,img,2,scale);
    }
}

template<typename State>
void drawLines(std::vector<State> const& states, int W, int H, cv::Mat &img, double scale=1.0)
{
    if(img.empty())
    {
        img = cv::Mat::zeros(cv::Size(W*scale,H*scale), CV_8UC3);
    }

    for(int i=0; i<static_cast<int>(states.size())-1; ++i)
    {
        drawLine(states[i], states[i+1],W,H,img,1,scale);
    }
}

template<typename StateActionState>
void drawActions(std::vector<StateActionState> const& actions, int W, int H, cv::Mat &img, double scale=1.0)
{
    if(img.empty())
    {
        img = cv::Mat::zeros(cv::Size(W*scale,H*scale), CV_8UC3);
    }

    for(auto sas : actions)
    {
        drawLine(sas.from, sas.to,W,H,img,2,scale);
    }
}

template<class NODE>
void drawNode(NODE const* node, int x, int y, cv::Mat &img, double value_factor=1.0, double scale=1.0)
{
    if(node && node->visited)
    {
        typedef unsigned char pixel_t[3];

        for(int i=scale*x; i< std::min(int(scale*x+scale), img.cols) ; i++)
        {
            for(int j=scale*y;j < std::min(int(scale*y+scale), img.rows); j++)
            {
                int value = value_factor * node->g;
                value = std::min(255,value);
                img.at<pixel_t>(j,i)[2] = std::min(img.at<pixel_t>(j,i)[2]+char(100*double(value)/255),255);
            }
        }

    }

}

template<class NODE_FINAL, class NODE, class KEY>
void drawOpenList(std::vector<std::pair<KEY,NODE*> > const& nodes, int W, int H,
        cv::Mat &img, double value_factor=1.0, double scale=1.0)
{
    if(img.empty())
    {
        img = cv::Mat::zeros(cv::Size(W*scale,H*scale), CV_8UC3);
    }


    typename std::vector<std::pair<KEY,NODE*> >::const_iterator it = nodes.begin();
    typename std::vector<std::pair<KEY,NODE*> >::const_iterator it_e = nodes.end();

    for(;it!=it_e;it++)
    {
        NODE_FINAL const* node_m = static_cast<NODE_FINAL const*>(it->second);
        if(node_m->open)
            drawState(node_m->state,W,H,img,2,scale);

    }
}


template<class NODE>
void drawNodes(std::vector<NODE*> const& nodes, int W, int H, cv::Mat &img, double value_factor=1.0, double scale=1.0)
{
    if(img.empty())
    {
        img = cv::Mat::zeros(cv::Size(W*scale,H*scale), CV_8UC3);
    }



    typename std::vector<NODE*>::const_iterator it = nodes.begin();
    typename std::vector<NODE*>::const_iterator it_e = nodes.end();

    for(;it!=it_e;it++)
    {
        NODE const* node_m = *it;

        if(node_m && node_m->visited)
            drawNode(node_m, node_m->state.x, node_m->state.y,img,value_factor,scale);

    }
}

template<class NODE>
void drawParents(std::vector<NODE*> const& nodes, int W, int H, cv::Mat &img, double value_factor=1.0, double scale=1.0)
{
    if(img.empty())
    {
        img = cv::Mat::zeros(cv::Size(W*scale,H*scale), CV_8UC3);
    }



    typename std::vector<NODE*>::const_iterator it = nodes.begin();
    typename std::vector<NODE*>::const_iterator it_e = nodes.end();

    for(;it!=it_e;it++)
    {
        NODE const* node_m = *it;

        if(node_m && node_m->visited)
        {
            if(node_m->parent.neighbor )
            {
                drawLine(node_m->state, node_m->parent.state,W,H,img,2,scale);
            }
        }

    }
}


template<class NODE>
void drawNodesInitialized(std::vector<NODE*> const& nodes, int W, int H, cv::Mat &img, double value_factor=1.0, double scale=1.0)
{
    if(img.empty())
    {
        img = cv::Mat::zeros(cv::Size(scale*W,scale*H), CV_8UC3);
    }

    typedef unsigned char pixel_t[3];

    for(int i=0; i<scale*W*scale*H; ++i)
    {
        int mi = i/int(scale*W);
        int mj = i%int(scale*W);
        int mid = int(mi/scale)*W+int(mj/scale);

        NODE const* node_m = nodes[mid];

        if(node_m && node_m->visited)
        {

            int value = value_factor * node_m->g;
            value = std::min(255,value);
            img.at<pixel_t>(i)[0] = value;
        }
    }
}

template<typename Algorithm>
void drawAlgorithm(   std::string name,
        unsigned char* map, int W, int H,
        Domain::State start_state, Domain::State goal_state,
        Algorithm &algorithm, double scale, std::vector<std::string> text = std::vector<std::string>())
{
    cv::Mat img;
    drawFancyMap(map, W,H, img, scale);
    drawCircle(start_state, W, H, img, scale, 3);
    drawCross(goal_state, W, H, img, scale, 2);


    drawNodes(algorithm.search_space->nodes(), W, H, img, 1.0, scale);
    drawOpenList<typename Algorithm::Node_t>(algorithm.open_list->heap(), W, H, img, 1.0, scale);
    drawLines(algorithm.getStatePath(), W, H, img, scale);

    if(!text.empty())
    {
        drawText(text, W, H, img, scale);
    }

    cv::imshow(name.c_str(),img);
}

}//namespace debug_cv
}//namespace movingai_benchmark
#endif /* MOVINGAI_BENCHMARK_DEBUG_OPENCV_H */
