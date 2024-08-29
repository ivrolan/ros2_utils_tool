add_library (ros_tools_config_widgets INTERFACE)

target_include_directories (ros_tools_config_widgets
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(ros_tools_config_widgets INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagWidget.hpp
)

target_link_libraries(ros_tools_config_widgets
    INTERFACE Qt::Widgets ros_tools_utils
) 
