add_library (ros_tools_thread INTERFACE)

target_include_directories (ros_tools_thread
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(ros_tools_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/EncodingThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/EncodingThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/WriteToBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/WriteToBagThread.hpp
)

target_link_libraries(ros_tools_thread
    INTERFACE ${OpenCV_LIBS} Qt::Widgets ros_tools_utils
)
