#-------------------------------------------------------------------------------
# Testing Publisher Module
#-------------------------------------------------------------------------------
project(test_publisher C CXX)

set(SOURCES
    test_publisher.cpp
)

set(PRIVATE_LIBRARIES
    ${PRIVATE_LIBRARIES})

#-------------------------------------------------------------------------------
# Includes
#-------------------------------------------------------------------------------
set(PRIVATE_INCLUDES ${OpenCV_INCLUDE_DIRS} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${cxxopts_INCLUDE_DIRS} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${rclcpp_INCLUDE_DIRS} ${PRIVATE_INCLUDES})
set(PRIVATE_INCLUDES ${TENSORRT_BASE_DIR} ${PRIVATE_INCLUDES})

#-------------------------------------------------------------------------------
# Libraries
#-------------------------------------------------------------------------------
set(PRIVATE_LIBRARIES ${OpenCV_LIBS} ${PRIVATE_LIBRARIES})
set(PRIVATE_LIBRARIES ${rclcpp_LIBRARIES} ${PRIVATE_LIBRARIES})
set(PRIVATE_LIBRARIES ${std_msgs_LIBRARIES} ${PRIVATE_LIBRARIES})
set(PRIVATE_LIBRARIES ${sensor_msgs_LIBRARIES} ${PRIVATE_LIBRARIES})
set(PRIVATE_LIBRARIES tensorrt_base_static ${PRIVATE_LIBRARIES})

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}
                      PRIVATE
                      ${PRIVATE_LIBRARIES})
target_include_directories(${PROJECT_NAME}
                           PRIVATE
                           ${PRIVATE_INCLUDES})

#-------------------------------------------------------------------------------
# Testing Subscriber Module
#-------------------------------------------------------------------------------
project(test_subscriber C CXX)

set(SOURCES
    test_subscriber.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}
                      PRIVATE
                      ${PRIVATE_LIBRARIES})
target_include_directories(${PROJECT_NAME}
                           PRIVATE
                           ${PRIVATE_INCLUDES})
