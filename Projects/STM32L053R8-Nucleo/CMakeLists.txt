##-------------Application choice for STM32L053R8-Nucleo-------------##
#
#
project(STM32L053R8-Nucleo)
cmake_minimum_required(VERSION 3.6)


#The following are different target builds for different Application choice, the only changes to the code in each if() condition are the directories

if(APPLICATION STREQUAL AT_Master)

    #---------------------------------------------------------------------------------------
    # AT_Slave target build
    #---------------------------------------------------------------------------------------
    list(APPEND ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/Core/src/mlm32l0xx_hal_msp.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/Core/src/mlm32l0xx_hw.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/Core/src/mlm32l0xx_it.c"

    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/at.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/command.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/debug.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/hw_gpio.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/hw_rtc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/hw_spi.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/lora.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/test_rf.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/tiny_sscanf.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/tiny_vsnprintf.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/src/vcom.c"

    )    

	#Adding the target , it can be add library or add executable
	add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

	target_compile_definitions(${PROJECT_NAME} PUBLIC -DUSE_HAL_DRIVER -DSTM32L072xx)

	# Add define if debbuger support is enabled
	target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_DEBUGGER}>:USE_DEBUGGER>)

	# Add define if radio debug pins support is enabled
	target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_RADIO_DEBUG}>:USE_RADIO_DEBUG>)

	target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/Core/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/LoRaWAN/App/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave/SW4STM32
        
   # $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
   # $<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>
   # $<TARGET_PROPERTY:radio,INTERFACE_INCLUDE_DIRECTORIES>
   # $<TARGET_PROPERTY:peripherals,INTERFACE_INCLUDE_DIRECTORIES>
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)

elseif(APPLICATION STREQUAL End_Node)

    #---------------------------------------------------------------------------------------
    # End Node target build
    #---------------------------------------------------------------------------------------
    list(APPEND ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/Core/src/mlm32l0xx_hal_msp.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/Core/src/mlm32l0xx_hw.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/Core/src/mlm32l0xx_it.c"
    
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/LoRaWAN/App/src/bsp.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/LoRaWAN/App/src/debug.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/LoRaWAN/App/src/hw_gpio.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/LoRaWAN/App/src/hw_rtc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/LoRaWAN/App/src/hw_spi.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/LoRaWAN/App/src/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/LoRaWAN/App/src/vcom.c"

    )    

	#Adding the target , it can be add library or add executable
	add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

	target_compile_definitions(${PROJECT_NAME} PUBLIC -DUSE_HAL_DRIVER -DSTM32L072xx)

	# Add define if debbuger support is enabled
	target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_DEBUGGER}>:USE_DEBUGGER>)

	# Add define if radio debug pins support is enabled
	target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_RADIO_DEBUG}>:USE_RADIO_DEBUG>)

	target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/Core/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/LoRaWAN/App/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node/SW4STM32
        
   # $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
   # $<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>
   # $<TARGET_PROPERTY:radio,INTERFACE_INCLUDE_DIRECTORIES>
   # $<TARGET_PROPERTY:peripherals,INTERFACE_INCLUDE_DIRECTORIES>
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)

else() #If application chosen is Ping Pong

    #---------------------------------------------------------------------------------------
    # Ping Pong target build
    #---------------------------------------------------------------------------------------
    list(APPEND ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/Core/src/mlm32l0xx_hal_msp.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/Core/src/mlm32l0xx_hw.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/Core/src/mlm32l0xx_it.c"
    
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/LoRaWAN/App/src/debug.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/LoRaWAN/App/src/hw_gpio.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/LoRaWAN/App/src/hw_rtc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/LoRaWAN/App/src/hw_spi.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/LoRaWAN/App/src/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/LoRaWAN/App/src/vcom.c"

    )    

	#Adding the target , it can be add library or add executable
	add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

	target_compile_definitions(${PROJECT_NAME} PUBLIC -DUSE_HAL_DRIVER -DSTM32L072xx)

	# Add define if debbuger support is enabled
	target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_DEBUGGER}>:USE_DEBUGGER>)

	# Add define if radio debug pins support is enabled
	target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_RADIO_DEBUG}>:USE_RADIO_DEBUG>)

	target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/Core/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/LoRaWAN/App/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong/SW4STM32
        
   # $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
   # $<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>
   # $<TARGET_PROPERTY:radio,INTERFACE_INCLUDE_DIRECTORIES>
   # $<TARGET_PROPERTY:peripherals,INTERFACE_INCLUDE_DIRECTORIES>
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
endif()

    