##-------------B-L072Z-LRWAN1-End_Node-------------##
#
#
project(B-L072Z-LRWAN1-End_Node)
cmake_minimum_required(VERSION 3.6)
enable_language(ASM)
#Specify the project's source files
file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/src/mlm32l0xx_hal_msp.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/src/mlm32l0xx_hw.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/src/mlm32l0xx_it.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/src/bsp.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/src/debug.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/src/hw_gpio.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/src/hw_rtc.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/src/hw_spi.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/src/main.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/src/vcom.c"  
    "${CMAKE_CURRENT_SOURCE_DIR}/SW4STM32/startup_stm32l072xx.s" 
      
)
set_property(SOURCE startup_stm32l072xx.s PROPERTY LANGUAGE C)

#Adding the target , it can be add library or add executable
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES}
                            $<TARGET_OBJECTS:middlewares>
                            $<TARGET_OBJECTS:Components>
                            $<TARGET_OBJECTS:BSP-B-L072Z-LRWAN1>
                            $<TARGET_OBJECTS:BSP-CMWX1ZZABZ-0xx>
                            $<TARGET_OBJECTS:BSP-X_NUCLEO_IKS01A1>
                            $<TARGET_OBJECTS:CMSIS-STM32L0xx>
                            $<TARGET_OBJECTS:sx1276>
                            $<TARGET_OBJECTS:STM32L0xx_HAL_Driver>
                            )
	
# Add define if debbuger support is enabled
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_DEBUGGER}>:USE_DEBUGGER>)
	
# Add define if radio debug pins support is enabled
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_RADIO_DEBUG}>:USE_RADIO_DEBUG>)

#Define the drivers instead of uncommenting them in CMSIS/Device/ST/STM32L0xx/Include/stm32L0xx.h
#if(bl072z STREQUAL T})
	#target_compile_definitions(${PROJECT_NAME} PUBLIC -DUSE_HAL_DRIVER -DSTM32L072xx)
	#include(stm32l0)
#endif()

add_dependencies(${PROJECT_NAME} middlewares)
add_dependencies(${PROJECT_NAME} BSP-B-L072Z-LRWAN1)
add_dependencies(${PROJECT_NAME} BSP-CMWX1ZZABZ-0xx)
add_dependencies(${PROJECT_NAME} Components)
add_dependencies(${PROJECT_NAME} BSP-X_NUCLEO_IKS01A1)
add_dependencies(${PROJECT_NAME} CMSIS-STM32L0xx)
add_dependencies(${PROJECT_NAME} sx1276)
add_dependencies(${PROJECT_NAME} STM32L0xx_HAL_Driver)


#Add all the header files
target_include_directories( ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/inc
	$<TARGET_PROPERTY:middlewares,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:BSP-B-L072Z-LRWAN1,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:BSP-CMWX1ZZABZ-0xx,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:Components,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:BSP-X_NUCLEO_IKS01A1,INTERFACE_INCLUDE_DIRECTORIES>    
	$<TARGET_PROPERTY:CMSIS-STM32L0xx,INTERFACE_INCLUDE_DIRECTORIES>    
	$<TARGET_PROPERTY:STM32L0xx_HAL_Driver,INTERFACE_INCLUDE_DIRECTORIES>  
	$<TARGET_PROPERTY:sx1276,INTERFACE_INCLUDE_DIRECTORIES>      
	$<TARGET_PROPERTY:STM32L0xx_HAL_Driver,INTERFACE_INCLUDE_DIRECTORIES> 
)

target_include_directories( middlewares PUBLIC)
target_include_directories( BSP-B-L072Z-LRWAN1 PUBLIC)
target_include_directories( BSP-CMWX1ZZABZ-0xx PUBLIC)
target_include_directories( Components PUBLIC)
target_include_directories( BSP-X_NUCLEO_IKS01A1 PUBLIC)
target_include_directories( CMSIS-STM32L0xx PUBLIC)
target_include_directories( sx1276 PUBLIC)
target_include_directories( STM32L0xx_HAL_Driver PUBLIC)



#Specify the C standard whose features are requested to build this target
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
