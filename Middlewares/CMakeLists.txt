##-------------Middelwares CMakLists.txt-------------##
#
#
project(middlewares)
cmake_minimum_required(VERSION 3.6)

#Specify the project's source files
file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Conf/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Core/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Crypto/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/region/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Utilities/*.c"
    
    
)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

add_dependencies(${PROJECT_NAME} BSP-B-L072Z-LRWAN1)
add_dependencies(${PROJECT_NAME} BSP-CMWX1ZZABZ-0xx)
add_dependencies(${PROJECT_NAME} Components)
add_dependencies(${PROJECT_NAME} BSP-X_NUCLEO_IKS01A1)
add_dependencies(${PROJECT_NAME} CMSIS-STM32L0xx)
add_dependencies(${PROJECT_NAME} sx1276)
add_dependencies(${PROJECT_NAME} STM32L0xx_HAL_Driver)

#Add all the header files
target_include_directories( ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Conf/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Core
    ${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Crypto
    ${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac
    ${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/region
    ${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Phy
    ${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Utilities
    
    $<TARGET_PROPERTY:B-L072Z-LRWAN1-End_Node,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:BSP-B-L072Z-LRWAN1,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:BSP-CMWX1ZZABZ-0xx,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:Components,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:BSP-X_NUCLEO_IKS01A1,INTERFACE_INCLUDE_DIRECTORIES>    
	$<TARGET_PROPERTY:CMSIS-STM32L0xx,INTERFACE_INCLUDE_DIRECTORIES>    
	$<TARGET_PROPERTY:STM32L0xx_HAL_Driver,INTERFACE_INCLUDE_DIRECTORIES>  
	$<TARGET_PROPERTY:sx1276,INTERFACE_INCLUDE_DIRECTORIES>      
)

#Specify the C standard whose features are requested to build this target
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
