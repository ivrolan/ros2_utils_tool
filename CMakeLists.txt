cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(mediassist4_ros_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(${Qt6_FOUND})
    find_package(QT NAMES Qt6 COMPONENTS Widgets REQUIRED)
    find_package(Qt6 COMPONENTS Widgets REQUIRED)
else()
    find_package(QT NAMES Qt5 COMPONENTS Widgets REQUIRED)
    find_package(Qt5 COMPONENTS Widgets REQUIRED)
endif()

add_subdirectory(src)

add_executable(tool_app
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp resources/resources.qrc
)
add_executable(tool_bag_to_video
    ${CMAKE_CURRENT_LIST_DIR}/src/BagToVideo.cpp
)

ament_target_dependencies(tool_app cv_bridge rclcpp rosbag2_cpp sensor_msgs std_msgs)
ament_target_dependencies(tool_bag_to_video cv_bridge rclcpp rosbag2_cpp sensor_msgs)

target_link_libraries(tool_app
    Qt::Widgets ros_tools_ros ros_tools_ui
)
target_link_libraries(tool_bag_to_video
    Qt::Widgets ros_tools_main ros_tools_utils
)

install(TARGETS
    tool_app
    DESTINATION lib/${PROJECT_NAME}/
)
install(TARGETS
    tool_bag_to_video
    DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
