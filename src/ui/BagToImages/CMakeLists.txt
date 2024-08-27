add_library (ros_tools_bag_to_images INTERFACE)

target_include_directories (ros_tools_bag_to_images
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(ros_tools_bag_to_images INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesWidget.hpp
)

target_link_libraries(ros_tools_bag_to_images
    INTERFACE Qt::Widgets ros_tools_utils
) 
