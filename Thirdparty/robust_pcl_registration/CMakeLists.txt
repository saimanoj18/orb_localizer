cmake_minimum_required(VERSION 2.8)
project(robust_pcl_registration)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(HDRS_RICP
  robust_pcl_registration/gicp.h
  robust_pcl_registration/pda.h
  robust_pcl_registration/point_cloud_registration.h
  robust_pcl_registration/point_cloud_registration_params.hpp
  robust_pcl_registration/probabilistic_weight.hpp
  robust_pcl_registration/weight_updater_callback.hpp
  robust_pcl_registration/error_term.hpp)
#  robust_pcl_registration/utils.hpp)
set(SRCS_RICP
  robust_pcl_registration/gicp.cc
  robust_pcl_registration/pda.cc
  robust_pcl_registration/point_cloud_registration.cc)

#find_package(gflags REQUIRED)
#find_package(glog REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
IF(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
ELSE(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
ENDIF(EIGEN3_FOUND)
find_package(Ceres REQUIRED)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${EIGEN3_INCLUDE_DIR}
${CERES_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${SRCS_RICP})
target_link_libraries(${PROJECT_NAME} 
${EIGEN3_LIBS}
${CERES_LIBRARIES}
${PCL_LIBRARIES})

