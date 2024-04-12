add_subdirectory(thread)

add_library (ros_tools_main INTERFACE)

target_include_directories (ros_tools_main
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(ros_tools_main INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/VideoEncoder.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoEncoder.cpp
)

target_link_libraries(ros_tools_main
    INTERFACE ${OpenCV_LIBS}
)
