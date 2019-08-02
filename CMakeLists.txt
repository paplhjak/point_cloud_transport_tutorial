cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_transport_tutorial)

find_package(catkin REQUIRED COMPONENTS point_cloud_transport sensor_msgs rosbag)

catkin_package(CATKIN_DEPENDS point_cloud_transport sensor_msgs rosbag)

include_directories(include ${catkin_INCLUDE_DIRS})

# publisher
add_executable(publisher_test src/my_publisher.cpp)
add_dependencies(publisher_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(publisher_test ${catkin_LIBRARIES})


# subscriber
add_executable(subscriber_test src/my_subscriber.cpp)
add_dependencies(subscriber_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(subscriber_test ${catkin_LIBRARIES})


# Mark executables and/or libraries for installation
install(TARGETS publisher_test subscriber_test
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

