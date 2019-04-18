##-------------CMSIS/STM32L0xx-------------##
#
#
project(CMSIS-STM32L0xx)
cmake_minimum_required(VERSION 3.6)

#Specify the project's source files
file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/Templates/system_stm32l0xx.c"
    #"${CMAKE_CURRENT_SOURCE_DIR}/Source/Templates/arm/*.s"
     #"${CMAKE_CURRENT_SOURCE_DIR}/Source/Templates/arm/startup_stm32l072xx.s"
    #"${CMAKE_CURRENT_SOURCE_DIR}/Source/Templates/gcc/*.s"   
    #"${CMAKE_CURRENT_SOURCE_DIR}/Source/Templates/iar/*.s"
#    "${CMAKE_CURRENT_SOURCE_DIR}/../../../DSP_Lib/Source/BasicMathFunctions/*.c"
#    "${CMAKE_CURRENT_SOURCE_DIR}/../../../DSP_Lib/Source/CommonTables/*.c"   
#    "${CMAKE_CURRENT_SOURCE_DIR}/../../../DSP_Lib/Source/ComplexMathFunctions/*.c"   
#    "${CMAKE_CURRENT_SOURCE_DIR}/../../../DSP_Lib/Source/ControllerFunctions/*.c"   
#    "${CMAKE_CURRENT_SOURCE_DIR}/../../../DSP_Lib/Source/FastMathFunctions/*.c"   
#    "${CMAKE_CURRENT_SOURCE_DIR}/../../../DSP_Lib/Source/FilteringFunctions/*.c"   
#    "${CMAKE_CURRENT_SOURCE_DIR}/../../../DSP_Lib/Source/MatrixFunctions/*.c"   
#    "${CMAKE_CURRENT_SOURCE_DIR}/../../../DSP_Lib/Source/StatisticsFunctions/*.c"   
#    "${CMAKE_CURRENT_SOURCE_DIR}/../../../DSP_Lib/Source/SupportFunctions/*.c"   
#    "${CMAKE_CURRENT_SOURCE_DIR}/../../../DSP_Lib/Source/TransformFunctions/*.c"   
       
)
set_property(SOURCE startup_stm32l072xx.s PROPERTY LANGUAGE C)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

#Add all the header files
target_include_directories( ${PROJECT_NAME} PUBLIC
     ${CMAKE_CURRENT_SOURCE_DIR}/Include
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Include
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../RTOS/Template
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Documentation/Core/html
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Documentation/Core/html/search
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Documentation/DSP/html
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Documentation/DSP/html/search
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Documentation/General/html
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Lib/ARM
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../Lib/GCC
     
          
     $<TARGET_PROPERTY:middlewares,INTERFACE_INCLUDE_DIRECTORIES>
     $<TARGET_PROPERTY:B-L072Z-LRWAN1-End_Node,INTERFACE_INCLUDE_DIRECTORIES>
)

#Specify the C standard whose features are requested to build this target
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)

