add_library (ros_tools_utils INTERFACE)

target_include_directories (ros_tools_utils
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(ros_tools_utils INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/UtilsGeneral.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsGeneral.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsROS.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsROS.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsUI.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsUI.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoEncoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoEncoder.hpp
)

target_link_libraries(ros_tools_utils
    INTERFACE ${OpenCV_LIBS} Qt::Widgets
)
