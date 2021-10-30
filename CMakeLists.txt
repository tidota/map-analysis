SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(map-analysis)

find_package(octomap REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#file(GLOB SRC_FILES *.cpp)
#foreach (src ${SRC_FILES})
#  get_filename_component(target ${src} NAME_WE)
#  add_executable(${target} ${src})
#  target_link_libraries(${target} yaml-cpp ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})
#endforeach ()

add_executable(eval eval.cpp)
target_link_libraries(eval yaml-cpp ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(count count.cpp)
target_link_libraries(count yaml-cpp ${OCTOMAP_LIBRARIES})

add_custom_target(settings ALL COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/settings.yaml settings.yaml)
