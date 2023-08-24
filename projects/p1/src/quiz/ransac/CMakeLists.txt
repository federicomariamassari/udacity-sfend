cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.11 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# 2-dimensional RANSAC
add_executable (quizRansac ransac2d.cpp ../../render/render.cpp)
target_link_libraries (quizRansac ${PCL_LIBRARIES})

# 3-dimensional RANSAC
add_executable (quizRansac3d ransac3d.cpp ../../render/render.cpp)
target_link_libraries (quizRansac3d ${PCL_LIBRARIES})
