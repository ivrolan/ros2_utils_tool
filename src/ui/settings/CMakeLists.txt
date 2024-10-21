add_library (rt_settings INTERFACE)

target_include_directories (rt_settings
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(rt_settings INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/AdvancedParamSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/AdvancedParamSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagParamSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagParamSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicParamSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicParamSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagParamSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagParamSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ImageParamSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ImageParamSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoParamSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoParamSettings.hpp
)

target_link_libraries(rt_settings
    INTERFACE Qt::Widgets
)
