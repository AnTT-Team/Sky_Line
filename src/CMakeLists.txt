# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS "main.cpp" "ble_nimble_uart.cpp" "ble_params.cpp" "ble_table.cpp"
    INCLUDE_DIRS "."
 )


