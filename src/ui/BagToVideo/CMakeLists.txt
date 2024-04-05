add_library (ros_tools_bag_to_video INTERFACE)

target_include_directories (ros_tools_bag_to_video
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(ros_tools_bag_to_video INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ProgressWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ProgressWidget.hpp
)

target_link_libraries(ros_tools_bag_to_video
    INTERFACE Qt::Widgets ros_tools_main ros_tools_utils
) 
