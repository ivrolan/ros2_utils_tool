find_package(catch_ros2 REQUIRED)

add_executable(tool_tests
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp

    ${CMAKE_CURRENT_LIST_DIR}/ui/settings/SettingsTest.cpp

    ${CMAKE_CURRENT_LIST_DIR}/utils/UtilsCLITest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils/UtilsGeneralTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils/UtilsROSTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils/UtilsSettingsTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils/UtilsUITest.cpp
)

ament_target_dependencies(tool_tests rclcpp rosbag2_cpp sensor_msgs std_msgs)

target_link_libraries(tool_tests
    catch_ros2::catch_ros2_with_main Qt::Widgets rt_settings rt_utils
)
