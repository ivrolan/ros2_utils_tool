add_subdirectory(BagToVideo)

add_library (ros_tools_ui INTERFACE)

target_include_directories (ros_tools_ui
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(ros_tools_ui INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/MainWindow.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MainWindow.hpp
    ${CMAKE_CURRENT_LIST_DIR}/StartWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/StartWidget.hpp
)

target_link_libraries(ros_tools_ui
    INTERFACE Qt::Widgets ros_tools_bag_to_video ros_tools_utils
) 
