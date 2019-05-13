##-------------Middelwares CMakLists.txt-------------##
#
#
project(middlewares)
cmake_minimum_required(VERSION 3.6)

#Because APPLICATION PingPong only needs the subfolder Utilities
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Utilities)

#if(APPLICATION STREQUAL AT)
#	file(GLOB ${PROJECT_NAME}_SOURCES
#	"${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Core/lora-test.c"
#	"${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Conf/Src/*.c"
#    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Crypto/*.c"
#    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/*.c"
#    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/region/*.c"
#    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Utilities/*.c"
#	)
#	
#else()
#	file(GLOB ${PROJECT_NAME}_SOURCES
#	"${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Core/*.c"
#	"${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Conf/Src/*.c"
#    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Crypto/*.c"
#    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/*.c"
#    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/region/*.c"
#    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Utilities/*.c"
#    )
#endif()
#	
if(CRYPTO STREQUAL PearlIoT)
	file(GLOB ${PROJECT_NAME}_SOURCES
	"${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Core/*.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Conf/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Crypto/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/region/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Utilities/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Core/*.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Conf/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Crypto/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/region/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Utilities/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMac.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacAdr.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacClassB.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacCommands.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacCrypto-SE.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacFCntHandler.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacParser.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.c"
    )
    
elseif(CRYPTO STREQUAL Soft-se)
	file(GLOB ${PROJECT_NAME}_SOURCES
	"${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Core/*.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Conf/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Crypto/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/region/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Utilities/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMac.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacAdr.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacClassB.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacCommands.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacConfirmQueue.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacCrypto.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacFCntHandler.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacParser.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/LoRaWAN/Mac/LoRaMacSerializer.c"
    
    )
endif()


add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})
	
#---------------------------------------------------------------------------------------
# Libraries - Common libraries independent from choices
#---------------------------------------------------------------------------------------


add_dependencies(${PROJECT_NAME} Components)

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
    $<TARGET_PROPERTY:Components,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:${PROJECT}-${APPLICATION},INTERFACE_INCLUDE_DIRECTORIES>    
    
)

#---------------------------------------------------------------------------------------
# Libraries - Sensor boards
#---------------------------------------------------------------------------------------
if(SENSOR_BOARD STREQUAL X_NUCLEO_IKS01A1 AND NOT(APPLICATION STREQUAL AT))
	add_dependencies(${PROJECT_NAME} BSP-X_NUCLEO_IKS01A1)
	target_include_directories( ${PROJECT_NAME} PUBLIC
	$<TARGET_PROPERTY:BSP-X_NUCLEO_IKS01A1,INTERFACE_INCLUDE_DIRECTORIES>    	
	)
elseif(SENSOR_BOARD STREQUAL X_NUCLEO_IKS01A2 AND NOT(APPLICATION STREQUAL AT))
	add_dependencies(${PROJECT_NAME} BSP-X_NUCLEO_IKS01A2)
	target_include_directories( ${PROJECT_NAME} PUBLIC
	$<TARGET_PROPERTY:BSP-X_NUCLEO_IKS01A2,INTERFACE_INCLUDE_DIRECTORIES>
	)    	
endif()     

#---------------------------------------------------------------------------------------
# Libraries - Series
#---------------------------------------------------------------------------------------
if(SERIES STREQUAL STM32L0xx)
	add_dependencies(${PROJECT_NAME} CMSIS-STM32L0xx)	
	add_dependencies(${PROJECT_NAME} STM32L0xx_HAL_Driver)
	target_include_directories( ${PROJECT_NAME} PUBLIC
		$<TARGET_PROPERTY:CMSIS-STM32L0xx,INTERFACE_INCLUDE_DIRECTORIES>
		$<TARGET_PROPERTY:STM32L0xx_HAL_Driver,INTERFACE_INCLUDE_DIRECTORIES>  
	)  	
elseif(SERIES STREQUAL STM32L1xx)
	add_dependencies(${PROJECT_NAME} CMSIS-STM32L1xx)	
	add_dependencies(${PROJECT_NAME} STM32L1xx_HAL_Driver)
	target_include_directories( ${PROJECT_NAME} PUBLIC
		$<TARGET_PROPERTY:CMSIS-STM32L1xx,INTERFACE_INCLUDE_DIRECTORIES>
		$<TARGET_PROPERTY:STM32L1xx_HAL_Driver,INTERFACE_INCLUDE_DIRECTORIES>  
	)
elseif(SERIES STREQUAL STM32L4xx)
	add_dependencies(${PROJECT_NAME} CMSIS-STM32L4xx)	
	add_dependencies(${PROJECT_NAME} STM32L4xx_HAL_Driver)
	target_include_directories( ${PROJECT_NAME} PUBLIC
		$<TARGET_PROPERTY:CMSIS-STM32L4xx,INTERFACE_INCLUDE_DIRECTORIES>
		$<TARGET_PROPERTY:STM32L4xx_HAL_Driver,INTERFACE_INCLUDE_DIRECTORIES>  
	)  
endif()   

#---------------------------------------------------------------------------------------
# Libraries - Radio
#---------------------------------------------------------------------------------------   
if(RADIO STREQUAL sx126x)
	add_dependencies(${PROJECT_NAME} sx126x)
	target_include_directories( ${PROJECT_NAME} PUBLIC
		$<TARGET_PROPERTY:sx126x,INTERFACE_INCLUDE_DIRECTORIES>  
	)  	
elseif(RADIO STREQUAL sx1272)
	add_dependencies(${PROJECT_NAME} sx1272)
	target_include_directories( ${PROJECT_NAME} PUBLIC
		$<TARGET_PROPERTY:sx1272,INTERFACE_INCLUDE_DIRECTORIES>  
	)
elseif(RADIO STREQUAL sx1276)
	add_dependencies(${PROJECT_NAME} sx1276)
	target_include_directories( ${PROJECT_NAME} PUBLIC
		$<TARGET_PROPERTY:sx1276,INTERFACE_INCLUDE_DIRECTORIES>  
	)	
endif()



#Specify the C standard whose features are requested to build this target
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
