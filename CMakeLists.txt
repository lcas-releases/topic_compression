cmake_minimum_required(VERSION 2.8.3)
project(topic_compression)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# ==============================================
# Catkin setup
# ==============================================
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        genmsg
        std_msgs
        sensor_msgs
        message_generation
        cv_bridge
        image_transport
        pybind11_catkin
)

# ==============================================
# Python files setup
# ==============================================
#catkin_python_setup()


# ==============================================
# Service/Message files setup
# ==============================================

add_message_files(
        FILES
        CompressedDepthImage.msg
        CompressedImage.msg
        CompressionMeta.msg
)


generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)


# ==============================================
# Setup
# ==============================================
catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs message_runtime cv_bridge pybind11_catkin)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include/
)


set(HEADER_FILES include/rvl.hpp)
add_executable(run src/bidirectional.cpp ${HEADER_FILES})
add_dependencies(run topic_compression_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(run ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


pybind_add_module(topic_compression_lib MODULE src/python_wrapper.cpp)
add_dependencies(topic_compression_lib topic_compression_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

# ==============================================
# Install
# ==============================================
install(DIRECTORY "docker" DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS run topic_compression_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


catkin_add_nosetests(tests/test_compression.py
        DEPENDENCIES ${${PROJECT_NAME}_EXPORTED_TARGETS})
        
if (CATKIN_ENABLE_TESTING)
        catkin_add_nosetests(tests)
endif()