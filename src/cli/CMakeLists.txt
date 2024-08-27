add_library (ros_tools_cli INTERFACE)

target_include_directories (ros_tools_cli
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(ros_tools_cli INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToImages.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideo.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBag.cpp
)

target_link_libraries(ros_tools_cli
    INTERFACE Qt::Widgets ros_tools_thread ros_tools_utils
)
