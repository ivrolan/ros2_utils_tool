add_library (ros_tools_utils INTERFACE)

target_include_directories (ros_tools_utils
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(ros_tools_utils INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/Utils.hpp
    ${CMAKE_CURRENT_LIST_DIR}/Utils.cpp
)

target_link_libraries(ros_tools_utils
    INTERFACE Qt::Widgets
)
