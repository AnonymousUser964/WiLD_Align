cmake_minimum_required(VERSION 2.8.3)
project(wild_align)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  wildalign_msgs
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED serialization filesystem system program_options thread COMPONENTS timer)

add_message_files(
  DIRECTORY msg
  FILES
  keyFrame.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  savePCD.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  wildalign_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
  wildalign_msgs
)

# include directories
include_directories(
	include
  include/pairwise_consistency_maximization
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

link_libraries(${Boost_LIBRARIES})



include(FetchContent)

FetchContent_Declare(
pmc
GIT_REPOSITORY https://github.com/jingnanshi/pmc
)

FetchContent_MakeAvailable(pmc)



# Graph utils library
add_library(graph_utils
        src/pairwise_consistency_maximization/graph_utils/graph_utils_functions.cpp
)
target_link_libraries(graph_utils
        gtsam
)

# Robot local map library
add_library(robot_measurements
        src/pairwise_consistency_maximization/robot_measurements/robot_measurements.cpp
        src/pairwise_consistency_maximization/robot_measurements/interrobot_measurements.cpp
        src/pairwise_consistency_maximization/robot_measurements/robot_local_map.cpp
        )

target_link_libraries(robot_measurements
        graph_utils
        )

# Paiwise Consistency computation library
add_library(pairwise_consistency
   src/pairwise_consistency_maximization/pairwise_consistency/pairwise_consistency.cpp
)
target_link_libraries(pairwise_consistency
       robot_measurements
)

# Maximum Clique Solver
add_library(fast_max-clique_finder
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findClique.h
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/graphIO.h
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findClique.cpp
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp 
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/utils.cpp 
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/graphIO.cpp
)
target_compile_options(fast_max-clique_finder PRIVATE -w)

# Global map solver library
add_library(global_map_solver
   src/pairwise_consistency_maximization/global_map/global_map.cpp
)
target_link_libraries(global_map_solver
   graph_utils
   pairwise_consistency
   fast_max-clique_finder
)

# Inter Optimization
add_executable(${PROJECT_NAME}_interOptimization 
src/interOptimization.cpp 
src/Scancontext.cpp
src/keyframeContainer.cpp
src/pcmProcessing.cpp
src/rangeModule.cpp
src/TrajectoryEstimator.cpp
src/RigidAlignment.cpp
)
add_dependencies(${PROJECT_NAME}_interOptimization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_interOptimization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_interOptimization Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} global_map_solver graph_utils robot_measurements gtsam pmc)
