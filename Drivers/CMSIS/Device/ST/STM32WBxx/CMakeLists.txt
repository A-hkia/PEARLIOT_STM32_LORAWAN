##-------------CMSIS/STM32WBxx-------------##
#
#
project(CMSIS-STM32WBxx)
cmake_minimum_required(VERSION 3.6)

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/system_stm32wbxx.c"   
)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

add_dependencies(${PROJECT_NAME} STM32WBxx_HAL_Driver)

#Add all the header files
target_include_directories( ${PROJECT_NAME} PUBLIC
     ${CMAKE_CURRENT_SOURCE_DIR}/Include
     $<TARGET_PROPERTY:middlewares,INTERFACE_INCLUDE_DIRECTORIES>
     $<TARGET_PROPERTY:STM32WBxx_HAL_Driver,INTERFACE_INCLUDE_DIRECTORIES>
     $<TARGET_PROPERTY:${PROJECT}-${APPLICATION},INTERFACE_INCLUDE_DIRECTORIES>
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Include
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../RTOS/Template
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Documentation/Core/html
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Documentation/Core/html/search
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Documentation/DSP/html
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Documentation/DSP/html/search
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Documentation/General/html
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Lib/ARM
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Lib/GCC
)


#Specify the C standard whose features are requested to build this target
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)

