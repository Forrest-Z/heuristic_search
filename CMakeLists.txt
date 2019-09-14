cmake_minimum_required(VERSION 2.8.3)
project(heuristic_search)

set(CMAKE_BUILD_TYPE Release)

add_definitions("-std=c++11 -Wall")

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS system filesystem thread program_options REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )

include_directories(include)

set(heuristic_search_src
  src/heuristic_search/benchmark/Benchmark.cpp

  src/heuristic_search/loggers/Log.cpp

  src/search_domains/movingai_benchmark/Domain.cpp

  src/search_domains/movingai_benchmark/debug_cv/debug_cv.cpp
  src/search_domains/movingai_benchmark/debug_cv/DynamicAlgorithms.cpp

  src/search_domains/movingai_benchmark/loader/MapLoader.cpp
  src/search_domains/movingai_benchmark/loader/ScenarioLoader.cpp
  src/search_domains/movingai_benchmark/loader/ScenarioDirectoryLoader.cpp

  src/search_domains/movingai_benchmark/benchmark/AStarRun.cpp
  src/search_domains/movingai_benchmark/benchmark/Benchmark.cpp
  src/search_domains/movingai_benchmark/benchmark/DynamicExperiment.cpp
  src/search_domains/movingai_benchmark/benchmark/Freespace.cpp
  src/search_domains/movingai_benchmark/benchmark/RandomChanges.cpp
  src/search_domains/movingai_benchmark/benchmark/Shortcuts.cpp
  src/search_domains/movingai_benchmark/ProgramOptions.cpp
)

add_library(heuristic_search  SHARED
    ${heuristic_search_src}

)
target_link_libraries(heuristic_search
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
)


add_executable(movingai_dstar_extra_lite_demo
    ${heuristic_search_src}
    src/search_domains/movingai_benchmark/movingai_dstar_extra_lite_demo.cpp
)
target_link_libraries(movingai_dstar_extra_lite_demo
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
)

add_executable(movingai_exp_dynamic_random
    ${heuristic_search_src}
    src/search_domains/movingai_benchmark/experiments/movingai_exp_dynamic_random.cpp
)
target_link_libraries(movingai_exp_dynamic_random
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
)

add_executable(movingai_exp_dynamic_freespace
    ${heuristic_search_src}
    src/search_domains/movingai_benchmark/experiments/movingai_exp_dynamic_freespace.cpp
)
target_link_libraries(movingai_exp_dynamic_freespace
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
)

add_executable(movingai_exp_dynamic_shortcuts
    ${heuristic_search_src}
    src/search_domains/movingai_benchmark/experiments/movingai_exp_dynamic_shortcuts.cpp
)
target_link_libraries(movingai_exp_dynamic_shortcuts
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
)

add_executable(movingai_freespace
    ${heuristic_search_src}
    src/search_domains/movingai_benchmark/movingai_freespace.cpp
)
target_link_libraries(movingai_freespace
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
)

add_executable(movingai_freespace_benchmark
    ${heuristic_search_src}
    src/search_domains/movingai_benchmark/movingai_freespace_benchmark.cpp
)
target_link_libraries(movingai_freespace_benchmark
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
)


#############
## Install ##
#############

install (DIRECTORY include/heuristic_search DESTINATION include)
install (DIRECTORY include/search_domains DESTINATION include)
install (DIRECTORY config DESTINATION movingai_benchmark)
install (FILES scripts/download_movingai_maps_and_problems.sh DESTINATION movingai_benchmark
  PERMISSIONS OWNER_READ OWNER_EXECUTE)
install (TARGETS
    heuristic_search
    movingai_dstar_extra_lite_demo
    movingai_exp_dynamic_random
    movingai_exp_dynamic_freespace
    movingai_exp_dynamic_shortcuts
    movingai_freespace
    movingai_freespace_benchmark
  RUNTIME DESTINATION movingai_benchmark
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

#############
## Testing ##
#############

option(BUILD_TESTS "Builds gtests" OFF)

if(BUILD_TESTS)
# Add test cpp file
add_executable(test_heuristic_search
    test/heuristic_search/test_TestDomain.cpp
    test/test_heuristic_search.cpp
)

# Link test executable against gtest & gtest_main
target_link_libraries(test_heuristic_search
    heuristic_search
    gtest
    gtest_main
)

add_test(
    NAME test_heuristic_search
    COMMAND test_heuristic_search
)
endif()
