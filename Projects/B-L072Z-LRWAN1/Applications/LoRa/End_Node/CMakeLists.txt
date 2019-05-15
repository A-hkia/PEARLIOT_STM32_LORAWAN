##-------------B-L072Z-LRWAN1-End_Node-------------##
#
#
project(B-L072Z-LRWAN1-End_Node)
cmake_minimum_required(VERSION 3.6)

include (binutils-arm-none-eabi)

#Specify the project's source files
file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/src/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/SW4STM32/startup_stm32l072xx.s" 
      
)
#For it to be able to take the startup file into account
set_property(SOURCE startup_stm32l072xx.s PROPERTY LANGUAGE C)

#---------------------------------------------------------------------------------------
# Libraries - Common libraries independent from choices
#---------------------------------------------------------------------------------------

#Adding the target | we use add executable because this is not a library
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES}
	$<TARGET_OBJECTS:middlewares>
	$<TARGET_OBJECTS:BSP-B-L072Z-LRWAN1>
	$<TARGET_OBJECTS:BSP-CMWX1ZZABZ-0xx>
	$<TARGET_OBJECTS:CMSIS-STM32L0xx>
	$<TARGET_OBJECTS:STM32L0xx_HAL_Driver>
	$<TARGET_OBJECTS:Components>
	)
         
add_dependencies(${PROJECT_NAME} middlewares)
add_dependencies(${PROJECT_NAME} BSP-B-L072Z-LRWAN1)
add_dependencies(${PROJECT_NAME} BSP-CMWX1ZZABZ-0xx)
add_dependencies(${PROJECT_NAME} CMSIS-STM32L0xx)	
add_dependencies(${PROJECT_NAME} STM32L0xx_HAL_Driver)
add_dependencies(${PROJECT_NAME} Components)



#Add all the header files
target_include_directories( ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/inc
	$<TARGET_PROPERTY:middlewares,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:BSP-B-L072Z-LRWAN1,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:BSP-CMWX1ZZABZ-0xx,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:CMSIS-STM32L0xx,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:STM32L0xx_HAL_Driver,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:Components,INTERFACE_INCLUDE_DIRECTORIES>      
)


#---------------------------------------------------------------------------------------
# Libraries - Sensor boards
#---------------------------------------------------------------------------------------

if(SENSOR_BOARD STREQUAL X_NUCLEO_IKS01A1)
	target_compile_definitions(${PROJECT_NAME} PUBLIC -DSENSOR_ENABLED)
	target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:BSP-X_NUCLEO_IKS01A1>)
	add_dependencies(${PROJECT_NAME} BSP-X_NUCLEO_IKS01A1)
	target_include_directories( ${PROJECT_NAME} PUBLIC
		$<TARGET_PROPERTY:BSP-X_NUCLEO_IKS01A1,INTERFACE_INCLUDE_DIRECTORIES>  
	)  	
elseif(SENSOR_BOARD STREQUAL X_NUCLEO_IKS01A2)
	target_compile_definitions(${PROJECT_NAME} PUBLIC -DSENSOR_ENABLED)
	target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:BSP-X_NUCLEO_IKS01A2>)
	add_dependencies(${PROJECT_NAME} BSP-X_NUCLEO_IKS01A2)
	target_include_directories( ${PROJECT_NAME} PUBLIC
		$<TARGET_PROPERTY:BSP-X_NUCLEO_IKS01A2,INTERFACE_INCLUDE_DIRECTORIES>  
	)
endif()

#---------------------------------------------------------------------------------------
# Libraries - Radio
#---------------------------------------------------------------------------------------   
if(RADIO STREQUAL sx1276)
	target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:sx1276>)
	add_dependencies(${PROJECT_NAME} sx1276)
	target_include_directories( ${PROJECT_NAME} PUBLIC
		$<TARGET_PROPERTY:sx1276,INTERFACE_INCLUDE_DIRECTORIES>  
	)  	
else()
	message(FATAL_ERROR "You can only choose the sx1276 for the BL072Z-LRWAN1")
endif()


# Add define if debbuger support is enabled
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_DEBUGGER}>:USE_DEBUGGER>)

# Add define if radio debug pins support is enabled
#target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_RADIO_DEBUG}>:USE_RADIO_DEBUG>)

#Specify the C standard whose features are requested to build this target
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)

# Create output in hex and binary format
create_bin_output(${PROJECT_NAME})
create_hex_output(${PROJECT_NAME})
