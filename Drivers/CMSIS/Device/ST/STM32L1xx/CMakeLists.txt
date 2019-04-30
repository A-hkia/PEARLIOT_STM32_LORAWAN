##-------------CMSIS/STM32L1xx-------------##
#
#
project(CMSIS-STM32L1xx)
cmake_minimum_required(VERSION 3.6)

#Specify the project's source files
file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/Templates/system_stm32l1xx.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/Templates/arm/*.s"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/Templates/gcc/*.s"   
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/Templates/iar/*.s"   
)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})
#Add all the header files
target_include_directories( ${PROJECT_NAME} PUBLIC
     ${CMAKE_CURRENT_SOURCE_DIR}/Include
     $<TARGET_PROPERTY:${PROJECT}-${APPLICATION},INTERFACE_INCLUDE_DIRECTORIES>
)


#Specify the C standard whose features are requested to build this target
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)

