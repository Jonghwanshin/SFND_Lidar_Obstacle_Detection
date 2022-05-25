cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

if(APPLE)
    set(Qt5_DIR /opt/homebrew/Cellar/qt@5/5.15.3/lib/cmake/qt5)
endif()
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL CONFIG REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment 
                src/environment.cpp 
                src/render/render.cpp 
                src/processPointClouds.cpp
                src/clustering_pcl.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})


# add_subdirectory(src/quiz/ransac)
# add_subdirectory(src/quiz/cluster)

