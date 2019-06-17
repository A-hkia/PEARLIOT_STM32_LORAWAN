##-------------Application choice for BL072Z-------------##
#
#
project(STM32WB55-Nucleo)
cmake_minimum_required(VERSION 3.6)

set(SERIES STM32WBxx CACHE STRING "Serie chosen is STM32WBxx")


#Application
if(APPLICATION STREQUAL AT)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave)

elseif(APPLICATION STREQUAL End_Node)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node)

elseif(APPLICATION STREQUAL PingPong) 
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong)  
endif()

    