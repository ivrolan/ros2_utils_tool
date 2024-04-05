add_library (ros_tools_ros INTERFACE)

target_include_directories (ros_tools_ros
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(ros_tools_ros INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ROSHandler.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ROSHandler.cpp
)

target_link_libraries(ros_tools_ros
    INTERFACE Qt::Widgets
)
