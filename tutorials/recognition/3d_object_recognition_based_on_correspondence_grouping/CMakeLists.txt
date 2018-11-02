#CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
#PROJECT(correspondence_grouping)
#
#FIND_PACKAGE(PCL REQUIRED)
#INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
#LINK_DIRECTORIES(${PCL_LIBRARY_DIES})
#
#ADD_EXECUTABLE(${PROJECT_NAME} ${PROJECT_NAME})
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(correspondence_grouping)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (correspondence_grouping correspondence_grouping.cpp)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})