##-------------sx126x-------------##
#
#
project(sx126x)
cmake_minimum_required(VERSION 3.6)

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/radio.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/sx126x.c"   
)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

target_include_directories( ${PROJECT_NAME} PUBLIC
     ${CMAKE_CURRENT_SOURCE_DIR}
     $<TARGET_PROPERTY:middlewares,INTERFACE_INCLUDE_DIRECTORIES>
	 $<TARGET_PROPERTY:${PROJECT}-${APPLICATION},INTERFACE_INCLUDE_DIRECTORIES>
	 )

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
