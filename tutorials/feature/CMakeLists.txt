PROJECT(feature)

FIND_PACKAGE(PCL 1.2 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_INCLUDE_DIRS})


#normal_estimation
ADD_EXECUTABLE(normal_estimation src/normal_estimation src/normal_estimation_using_integral_images.cpp)
TARGET_LINK_LIBRARIES(normal_estimation ${PCL_LIBRARIES})

#normal_estimation_using_integral_images
set(INTEGRAL_IMAGES normal_estimation_using_integral_images)
ADD_EXECUTABLE(${INTEGRAL_IMAGES} src/${INTEGRAL_IMAGES})
TARGET_LINK_LIBRARIES(${INTEGRAL_IMAGES} ${PCL_LIBRARIES})
