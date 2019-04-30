##-------------Project choice and configuration-------------##
#
#
project(projects)
cmake_minimum_required(VERSION 3.6)


#---------------------------------------------------------------------------------------
# Project Options
#---------------------------------------------------------------------------------------

#Choice B-L072Z-LRWAN1
if(PROJECT STREQUAL B-L072Z-LRWAN1)
    # Configure toolchain for B-L072Z-LRWAN1
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/SW4STM32/mlm32l07x01/STM32L072CZYx_FLASH.ld)
    #The same linker script is used for the three applications (AT_Slave/End_Node/PingPong)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/B-L072Z-LRWAN1)
    include(stm32l072)
    
#Choice STM32L053R8-Nucleo
elseif(PROJECT STREQUAL STM32L053R8-Nucleo)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/STM32L053R8-Nucleo)
    # Configure toolchain for STM32L053R8-Nucleo
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32L053R8-Nucleo/Applications/LoRa/AT_Master/SW4STM32/I_NUCLEO_LRWAN1/STM32L053R8Tx_FLASH.ld)
    #The same linker script is used for the three applications (AT_Master/End_Node/PingPong)
    include(stm32l053)

#Choice STM32L073RZ-Nucleo
elseif(PROJECT STREQUAL STM32L073RZ-Nucleo)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/STM32L073RZ-Nucleo)
    # Configure toolchain for STM32L073RZ-Nucleo
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32L073RZ-Nucleo/Applications/LoRa/AT_Master/SW4STM32/I_NUCLEO_LRWAN1/STM32L073RZTx_FLASH.ld)
    #The same linker script is used for the three applications (AT_Master/End_Node/PingPong)
    include(stm32l073)
    
#Choice STM32L152RE-Nucleo
elseif(PROJECT STREQUAL STM32L152RE-Nucleo)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/STM32L152RE-Nucleo)
    # Configure toolchain for STM32L152RE-Nucleo
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32L152RE-Nucleo/Applications/LoRa/End_Node/SW4STM32/sx1261dvk1bas/STM32L152RETx_FLASH.ld)
    #The same linker script is used for the two applications (End_Node/PingPong)
    include(stm32l152)

#Choice STM32L476RG-Nucleo
elseif(PROJECT STREQUAL STM32L476RG-Nucleo)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/STM32L476RG-Nucleo)
    # Configure toolchain for STM32L476RG-Nucleo
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32L476RG-Nucleo/Applications/LoRa/End_Node/SW4STM32/sx1261dvk1bas/STM32L476RGTx_FLASH.ld)
    #The same linker script is used for the two applications (End_Node/PingPong)
endif()


