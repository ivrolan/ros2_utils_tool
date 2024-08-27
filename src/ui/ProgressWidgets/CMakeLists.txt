add_library (ros_tools_progress_widgets INTERFACE)

target_include_directories (ros_tools_progress_widgets
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(ros_tools_progress_widgets INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BasicProgressWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicProgressWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ImagesProgressWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ImagesProgressWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoProgressWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoProgressWidget.hpp
)

target_link_libraries(ros_tools_progress_widgets
    INTERFACE Qt::Widgets ros_tools_utils
) 
