add_library (ros_tools_video_to_bag INTERFACE)

target_include_directories (ros_tools_video_to_bag
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(ros_tools_video_to_bag INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagWidget.hpp
)

target_link_libraries(ros_tools_video_to_bag
    INTERFACE Qt::Widgets ros_tools_main ros_tools_utils
) 
