# Heuristic Search Library

Heuristic Search Library is the open-source C++ library with the generic implementations of several incremental search algorithms based on A\*:

Originally found on Sourceforge. See https://bitbucket.org/maciej_przybylski/heuristic_search/src/master/

* A\*
* Repeated A\*
* [D\*-Lite](https://pdfs.semanticscholar.org/9c09/36e2fba53deb0516c7ba596bfacfe9a038a1.pdf)
* [D\*-Lite (optimized)](https://pdfs.semanticscholar.org/9c09/36e2fba53deb0516c7ba596bfacfe9a038a1.pdf)
* [MPGAA\*](http://www.academia.edu/download/41586182/HernandezAB15.pdf)
* [D\* Extra Lite](https://amcs.uz.zgora.pl/?action=paper&paper=1366)


The library also includes the domain for the [Moving AI Pathfinding Benchmark](http://movingai.com/benchmarks/) that provides the collection of 2D grid-maps (artificial and from video-games) together with randomly selected problems. The benchmark problems can be run with an OpenCV-based visualization.

## Requirements
The library has only been tested on Ubuntu 14.04, but it is likely to work on other Linux systems. Following software is required to build and run the library:

* cmake
* Boost
* OpenCV
* wget and unzip (only for the script that downloads benchmark).

## Installing
To install Heuristic Search Library type following commands in a command line.

Clone git repository.

```
git clone https://bitbucket.org/maciej_przybylski/heuristic_search.git
```
Prepare and enter build directory.
```
cd heuristic_search
mkdir build
cd build
```
Configure cmake (you may want to change install directory).
```
cmake ../ -DCMAKE_INSTALL_PREFIX=~/heuristic_search_install

```
Make and install the project.
```
make -j 4
make install
```

### Downloading the [Moving AI data sets](http://movingai.com/benchmarks/)

Enter `movingai_benchmark` in the install directory.
```
cd ~/heuristic_search_install/movingai_benchmark
```
In the directory you will find the script `download_movingai_maps_and_problems.sh` that downloads the data sets: wc3, sc1, rooms, mazes and random (137MiB). To run the script type:
```
./download_movingai_maps_and_problems.sh
```

## Running
### Single problem demo

To run a demo, enter `movingai_benchmark` directory
```
cd ~/heuristic_search_install/movingai_benchmark
```
and type
```
./movingai_freespace --config-file config/movingai_benchmark/freespace_sample_config.cfg
```
This demo will run selected algorithms for the same single problem as specified in `config/movingai_benchmark/freespace_sample_config.cfg`.
```
algorithms          = D*ExtraLite  
algorithms          = D*ExtraLite.NoKm  
algorithms          = D*LiteOpt.
algorithms          = D*Lite
algorithms          = MPGAA*

map-file            = maps/wc3maps/battleground.map
start.x             = 133
start.y             = 127              
goal.x              = 371
goal.y              = 455

observation-range   = 10
preinitialized      = no
undirected          = no

visualization       = yes
vis-scale           = 1.5
vis-delay           = 5
vis-hide-text       = no
```

### Benchmark
To run the benchmark, enter `movingai_benchmark` directory
```
cd ~/heuristic_search_install/movingai_benchmark
```
and type
```
./movingai_freespace_benchmark --config-file config/movingai_benchmark/freespace_benchmark_sample_config.cfg
```
The benchmark will run selected algorithms for the number of problems from the data-set as specified in `config/movingai_benchmark/freespace_benchmark_sample_config.cfg`.
```
algorithms          = D*ExtraLite  
algorithms          = D*ExtraLite.NoKm  
algorithms          = D*LiteOpt.
algorithms          = D*Lite
algorithms          = MPGAA*

observation-range   = 10
preinitialized      = yes
undirected          = no

benchmark-dir       = .
test-name           = freespace_benchmark
problems-path       = wc3
problems-number     = 10
```
The benchmark assumes that directory specified in `benchmark-dir` contains `maps` and `problems` subdirectories (`download_movingai_maps_and_problems.sh` ensures this). The benchmark results will be saved in `results` subdirectory.

## License

By downloading, copying, installing or using the software you agree to this license.

                          License Agreement
                     For heuristic_search library
                       (3-clause BSD License)

Copyright (c) 2016, Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>,  Warsaw University of Technology. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
