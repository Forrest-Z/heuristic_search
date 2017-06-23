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


#include "search_domains/movingai_benchmark/debug_cv/debug_cv.h"
#include "search_domains/movingai_benchmark/Domain.h"


namespace movingai_benchmark{
namespace debug_cv{

void drawMap(unsigned char *map, int W, int H, cv::Mat &img, double scale, int color, unsigned char intens)
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

        if(map[mid]==movingai_benchmark::OUT_OF_BOUNDS
                || map[mid]==movingai_benchmark::OUT_OF_BOUNDS_O
                || map[mid]==movingai_benchmark::TREES)
        {
            if(color & 4 )
            {
                img.at<pixel_t>(i)[0] = std::max(map[mid], intens);
            }

            if(color & 2)
            {
                img.at<pixel_t>(i)[1] = std::max(map[mid], intens);
            }

            if(color & 1)
            {
                img.at<pixel_t>(i)[2] = std::max(map[mid], intens);
            }
        }
    }
}

void drawFancyMap(unsigned char *map, int W, int H, cv::Mat &img, double scale)
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

        if(map[mid]==movingai_benchmark::OUT_OF_BOUNDS
                || map[mid]==movingai_benchmark::OUT_OF_BOUNDS_O)
        {
            img.at<pixel_t>(i)[0] = 0;
            img.at<pixel_t>(i)[1] = 0;
            img.at<pixel_t>(i)[2] = 0;
        }

        if(map[mid]==movingai_benchmark::PASSABLE_TERRAIN)
        {
            img.at<pixel_t>(i)[0] = 114;
            img.at<pixel_t>(i)[1] = 131;
            img.at<pixel_t>(i)[2] = 152;
        }

        if(map[mid]==movingai_benchmark::PASSABLE_TERRAIN_G)
        {
            img.at<pixel_t>(i)[0] = 90;
            img.at<pixel_t>(i)[1] = 90;
            img.at<pixel_t>(i)[2] = 90;
        }

        if(map[mid]==movingai_benchmark::TREES)
        {
            img.at<pixel_t>(i)[0] = 25;
            img.at<pixel_t>(i)[1] = 150;
            img.at<pixel_t>(i)[2] = 94;
        }

        if(map[mid]==movingai_benchmark::SWAMP)
        {
            img.at<pixel_t>(i)[0] = 128;
            img.at<pixel_t>(i)[1] = 128;
            img.at<pixel_t>(i)[2] = 0;
        }

        if(map[mid]==movingai_benchmark::WATER)
        {
            img.at<pixel_t>(i)[0] = 255;
            img.at<pixel_t>(i)[1] = 0;
            img.at<pixel_t>(i)[2] = 0;
        }
    }
}


void drawPoint(int x, int y, cv::Mat &img, int channel, double scale)
{
    typedef unsigned char pixel_t[3];

    for(int i=scale*x;i<std::min(int(scale*x+scale), img.cols); i++)
    {
        for(int j=scale*y;j<std::min(int(scale*y+scale), img.rows); j++)
        {
            img.at<pixel_t>(j,i)[channel] = 255;
        }
    }

}

void drawState(Domain::State const& state, int W, int H, cv::Mat &img, int channel, double scale)
{
    drawPoint(state.x, state.y, img, channel, scale);
}

void drawCircle(int x, int y, cv::Mat &img, double scale, int size)
{
    cv::Scalar color(255.0,255.0,255.0);
    cv::circle(img,cv::Point(scale*x + scale/2,scale*y + scale/2),size,color);           
}

void drawCircle(Domain::State const& state, int W, int H, cv::Mat &img, double scale,int size)
{
    if(img.empty())
    {
        img = cv::Mat::zeros(cv::Size(W,H), CV_8UC3);
    }

    drawCircle(state.x, state.y, img, scale, size);

}

void drawLine(int from_x, int from_y, int to_x, int to_y, cv::Mat &img, int channel, double scale)
{
    cv::Scalar color(0.0,0.0,0.0);

    if(channel & 4 )
    {
        color[0] = 255.0;
    }

    if(channel & 2)
    {
        color[1] = 255.0;
    }

    if(channel & 1)
    {
        color[2] = 255.0;
    }

    cv::line(img, cv::Point(scale*from_x + scale/2,scale*from_y + scale/2),
                cv::Point(scale*to_x + scale/2,scale*to_y + scale/2), color, 1);

}

void drawLine(Domain::State const& from, Domain::State const& to, int W, int H, cv::Mat &img, int channel, double scale)
{
    if(img.empty())
    {
        img = cv::Mat::zeros(cv::Size(W,H), CV_8UC3);
    }

    drawLine(from.x, from.y, to.x, to.y, img, channel, scale);

}

void drawCross(int x, int y, int W, int H, cv::Mat &img, double scale, int size)
{

    cv::Scalar color(255.0,255.0,255.0);

    int cx = scale*x + scale/2;
    int cy = scale*y + scale/2;

    int from_x1 = std::max(cx-size,0);
    int from_y1 = std::max(cy-size,0);

    int to_x1 = std::min(cx+size,int(W*scale));
    int to_y1 = std::min(cy+size,int(H*scale));

    cv::line(img, cv::Point(from_x1, from_y1),
                cv::Point(to_x1, to_y1), color, 1);

    int from_x2 = std::max(cx-size,0);
    int from_y2 = std::min(cy+size,int(H*scale));

    int to_x2 = std::min(cx+size,int(W*scale));
    int to_y2 = std::max(cy-size,0);

    cv::line(img, cv::Point(from_x2, from_y2),
                cv::Point(to_x2, to_y2), color, 1);

}

void drawCross(Domain::State const& state, int W, int H, cv::Mat &img, double scale,int size)
{
    if(img.empty())
    {
        img = cv::Mat::zeros(cv::Size(W,H), CV_8UC3);
    }

    drawCross(state.x, state.y, W, H, img, scale, size);

}

void drawText(std::vector<std::string> text, int W, int H, cv::Mat &img, double scale)
{
    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;

    int interline = 7;
    int row = interline;

    for(int i=0; i<int(text.size()); ++i)
    {
        int baseline=0;
        cv::Size textSize = cv::getTextSize(text[i], fontFace,
                                    fontScale, thickness, &baseline);

        cv::Point textOrg(1,row+textSize.height);

        cv::putText(img, text[i], textOrg, fontFace, fontScale,
                cv::Scalar::all(255), thickness, 8);

        row += textSize.height+interline;
    }
}

}//namespace debug_cv
}//namespace movingai_benchmark
