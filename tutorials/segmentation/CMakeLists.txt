PROJECT(segmentation)

FIND_PACKAGE(PCL 1.2 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_INCLUDE_DIRS})


#euclidean_cluster_extraction
ADD_EXECUTABLE(euclidean_cluster_extraction src/euclidean_cluster_extraction ../feature/src/normal_estimation.cpp)
TARGET_LINK_LIBRARIES(euclidean_cluster_extraction ${PCL_LIBRARIES})
