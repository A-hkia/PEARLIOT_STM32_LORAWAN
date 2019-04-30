##-------------STM32L152RE-Nucleo-End_Node-------------##
#
#
project(STM32L152RE-Nucleo-End_Node)
cmake_minimum_required(VERSION 3.6)

include (binutils-arm-none-eabi)

#Specify the project's source files
file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/src/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/SW4STM32/startup_stm32l152xe.s" 
      
)
#For it to be able to take the startup file into account
set_property(SOURCE startup_stm32l152xe.s PROPERTY LANGUAGE C)


#---------------------------------------------------------------------------------------
# Libraries - Common libraries independent from choices
#---------------------------------------------------------------------------------------

#Adding the target | we use add executable because this is not a library
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES}
	$<TARGET_OBJECTS:middlewares>
	$<TARGET_OBJECTS:STM32L1xx_Nucleo>
	$<TARGET_OBJECTS:CMSIS-STM32L1xx>
	$<TARGET_OBJECTS:STM32L1xx_HAL_Driver>
	$<TARGET_OBJECTS:Components>
	
	)
         
add_dependencies(${PROJECT_NAME} middlewares)
add_dependencies(${PROJECT_NAME} STM32L1xx_Nucleo)
add_dependencies(${PROJECT_NAME} CMSIS-STM32L1xx)	
add_dependencies(${PROJECT_NAME} STM32L1xx_HAL_Driver)
add_dependencies(${PROJECT_NAME} Components)




#Add all the header files
target_include_directories( ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/LoRaWAN/App/inc
	$<TARGET_PROPERTY:middlewares,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:STM32L1xx_Nucleo,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:CMSIS-STM32L1xx,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:STM32L1xx_HAL_Driver,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:Components,INTERFACE_INCLUDE_DIRECTORIES>  
    
)


#---------------------------------------------------------------------------------------
# Libraries - Radio
#---------------------------------------------------------------------------------------   
if(RADIO STREQUAL sx126x)
	target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:sx126x>)
	target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:SX1261DVK1BAS>)
	#target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:SX1262DVK1CAS>)
	#target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:SX1262DVK1DAS>)
	

	add_dependencies(${PROJECT_NAME} sx126x)
	add_dependencies(${PROJECT_NAME} SX1261DVK1BAS)
	#add_dependencies(${PROJECT_NAME} SX1262DVK1CAS)
	#add_dependencies(${PROJECT_NAME} SX1262DVK1DAS)
	
	target_include_directories( ${PROJECT_NAME} PUBLIC 
	$<TARGET_PROPERTY:sx126x,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:SX1261DVK1BAS,INTERFACE_INCLUDE_DIRECTORIES>
	#$<TARGET_PROPERTY:SX1262DVK1CAS,INTERFACE_INCLUDE_DIRECTORIES>
	#$<TARGET_PROPERTY:SX1262DVK1DAS,INTERFACE_INCLUDE_DIRECTORIES>	
	)  	
	
elseif(RADIO STREQUAL sx1272)
	target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:sx1272>)
	target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:SX1272MB2DAS>)	

	add_dependencies(${PROJECT_NAME} sx1272)
	add_dependencies(${PROJECT_NAME} SX1272MB2DAS)
	
	target_include_directories( ${PROJECT_NAME} PUBLIC 
	$<TARGET_PROPERTY:sx1272,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:SX1272MB2DAS,INTERFACE_INCLUDE_DIRECTORIES>	
	)  
		
elseif(RADIO STREQUAL sx1276)
	target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:sx1276>)
	#target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:SX1276MB1LAS>)
	target_link_libraries(${PROJECT_NAME} $<TARGET_OBJECTS:SX1276MB1MAS>)	

	add_dependencies(${PROJECT_NAME} sx1276)
	#add_dependencies(${PROJECT_NAME} SX1276MB1LAS)
	add_dependencies(${PROJECT_NAME} SX1276MB1MAS)
	
	target_include_directories( ${PROJECT_NAME} PUBLIC 
	$<TARGET_PROPERTY:sx1276,INTERFACE_INCLUDE_DIRECTORIES>
	#$<TARGET_PROPERTY:SX1276MB1LAS,INTERFACE_INCLUDE_DIRECTORIES>	
	$<TARGET_PROPERTY:SX1276MB1MAS,INTERFACE_INCLUDE_DIRECTORIES>	
	)  
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
