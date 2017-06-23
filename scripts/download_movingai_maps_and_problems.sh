#!/bin/bash
cd movingai_benchmark
mkdir maps
mkdir problems

function download_map_set {
  cdir="$PWD"
  cd maps
  mkdir -p $1
  cd $1
  wget $2
  unzip map.zip
  rm map.zip
  cd "$cdir"/problems
  mkdir -p $3
  cd $3
  wget $4
  unzip scen.zip
  rm scen.zip
  cd "$cdir"
}

download_map_set wc3maps http://movingai.com/benchmarks/wc3maps512/map.zip wc3 http://movingai.com/benchmarks/wc3maps512/scen.zip
download_map_set sc1 http://movingai.com/benchmarks/sc1/map.zip sc1 http://movingai.com/benchmarks/sc1/scen.zip
download_map_set mazes http://movingai.com/benchmarks/mazes1/map.zip mazes/mazes1 http://movingai.com/benchmarks/mazes1/scen.zip
download_map_set mazes http://movingai.com/benchmarks/mazes4/map.zip mazes/mazes4 http://movingai.com/benchmarks/mazes4/scen.zip
download_map_set mazes http://movingai.com/benchmarks/mazes8/map.zip mazes/mazes8 http://movingai.com/benchmarks/mazes8/scen.zip
download_map_set mazes http://movingai.com/benchmarks/mazes16/map.zip mazes/mazes16 http://movingai.com/benchmarks/mazes16/scen.zip
download_map_set mazes http://movingai.com/benchmarks/mazes32/map.zip mazes/mazes32 http://movingai.com/benchmarks/mazes32/scen.zip
download_map_set random http://movingai.com/benchmarks/random10/map.zip random/random10 http://movingai.com/benchmarks/random10/scen.zip
download_map_set random http://movingai.com/benchmarks/random15/map.zip random/random15 http://movingai.com/benchmarks/random15/scen.zip
download_map_set random http://movingai.com/benchmarks/random20/map.zip random/random20 http://movingai.com/benchmarks/random20/scen.zip
download_map_set random http://movingai.com/benchmarks/random25/map.zip random/random25 http://movingai.com/benchmarks/random25/scen.zip
download_map_set random http://movingai.com/benchmarks/random30/map.zip random/random30 http://movingai.com/benchmarks/random30/scen.zip
download_map_set random http://movingai.com/benchmarks/random35/map.zip random/random35 http://movingai.com/benchmarks/random35/scen.zip
download_map_set random http://movingai.com/benchmarks/random40/map.zip random/random40 http://movingai.com/benchmarks/random40/scen.zip
download_map_set rooms http://movingai.com/benchmarks/room8/map.zip rooms/rooms8 http://movingai.com/benchmarks/room8/scen.zip
download_map_set rooms http://movingai.com/benchmarks/room16/map.zip rooms/rooms16 http://movingai.com/benchmarks/room16/scen.zip
download_map_set rooms http://movingai.com/benchmarks/room32/map.zip rooms/rooms32 http://movingai.com/benchmarks/room32/scen.zip
download_map_set rooms http://movingai.com/benchmarks/room64/map.zip rooms/rooms64 http://movingai.com/benchmarks/room64/scen.zip
