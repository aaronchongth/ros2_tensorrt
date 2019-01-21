#-------------------------------------------------------------------------------
project(ros2_base C CXX)

set(SOURCES
    ros2_tensorrt.cpp
)

set(PRIVATE_LIBRARIES
    ${PRIVATE_LIBRARIES})

#-------------------------------------------------------------------------------
# Includes
#-------------------------------------------------------------------------------
set(PRIVATE_INCLUDES ${OpenCV_INCLUDE_DIRS} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${cxxopts_INCLUDE_DIRS} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${eigen_INCLUDE_DIRS} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${CUDA_INCLUDE_DIRS} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${rclcpp_INCLUDE_DIRS} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${std_msgs_INCLUDE_DIRS} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${sensor_msgs_INCLUDE_DIRS} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${TENSORRT_BASE_DIR} ${PRIVATE_INCLUDES})

#-------------------------------------------------------------------------------
# Libraries
#-------------------------------------------------------------------------------
set(PRIVATE_LIBRARIES ${OpenCV_LIBS} ${PRIVATE_LIBRARIES})
set(PRIVATE_LIBRARIES ${CUDA_LIBRARIES} ${PRIVATE_LIBRARIES})
set(PRIVATE_LIBRARIES ${rclcpp_LIBRARIES} ${PRIVATE_LIBRARIES})
set(PRIVATE_LIBRARIES ${std_msgs_LIBRARIES} ${PRIVATE_LIBRARIES})
set(PRIVATE_LIBRARIES ${sensor_msgs_LIBRARIES} ${PRIVATE_LIBRARIES})
set(PRIVATE_LIBRARIES tensorrt_base_static ${PRIVATE_LIBRARIES})

#-------------------------------------------------------------------------------
# Final target as library
#-------------------------------------------------------------------------------
add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}
                      PRIVATE
                      ${PRIVATE_LIBRARIES})
target_include_directories(${PROJECT_NAME}
                           PRIVATE
                           ${PRIVATE_INCLUDES})

add_library("${PROJECT_NAME}_static" STATIC ${SOURCES})
target_link_libraries("${PROJECT_NAME}_static"
                      PUBLIC
                      ${PRIVATE_LIBRARIES})
target_include_directories("${PROJECT_NAME}_static"
                           PUBLIC
                           ${PRIVATE_INCLUDES})