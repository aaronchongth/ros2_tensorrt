#-------------------------------------------------------------------------------
project(tensorrt_base C CXX)

set(SOURCES
    tensorrt_base.cpp
    malloc_utilities.cpp
)

set(PRIVATE_LIBRARIES
    ${PRIVATE_LIBRARIES})

#-------------------------------------------------------------------------------
# Includes
#-------------------------------------------------------------------------------
set(PRIVATE_INCLUDES ${CUDA_INCLUDE_DIRS} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${TENSORRT_INCLUDE_DIR} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${rclcpp_INCLUDE_DIRS} ${PRIVATE_INCLUDES})

#-------------------------------------------------------------------------------
# Libraries
#-------------------------------------------------------------------------------
set(PRIVATE_LIBRARIES ${CUDA_LIBRARIES} ${PRIVATE_LIBRARIES})
set(PRIVATE_LIBRARIES ${TENSORRT_LIBRARY} ${PRIVATE_LIBRARIES})
set(PRIVATE_LIBRARIES ${rclcpp_LIBRARIES} ${PRIVATE_LIBRARIES})

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
