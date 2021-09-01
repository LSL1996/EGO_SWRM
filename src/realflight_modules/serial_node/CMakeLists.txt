cmake_minimum_required(VERSION 3.0.2)
project(serial_node)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        serial
        std_msgs
        sensor_msgs
        message_generation
)
add_message_files(FILES
        my_msg.msg
        rawpose.msg
)
generate_messages(DEPENDENCIES
        std_msgs
        sensor_msgs
        )
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        message_runtime
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        "mavlink_headers"
        "serial_node/cmake-build-debug/devel/include"
        "usr/local/include/eigen3"
)

add_executable( myserial src/myserialpub.cpp)
#add_executable(mavlink ${LIBCONFIG_MAVLINK_DIR})
#add_dependencies(my_msg my_msg_gencpp)
add_dependencies( myserial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})  #该项让编译器知道要注意编译顺序
#add_dependencies(myserial ${PROJECT_NAME}_generate_messages_cpp  ${PROJECT_NAME}_gencpp)
target_link_libraries(myserial ${catkin_LIBRARIES} )
