##-------------Application choice for BL072Z-------------##
#
#
project(B-L072Z-LRWAN1)
cmake_minimum_required(VERSION 3.6)

set(SERIES STM32L0xx CACHE STRING "Serie chosen is STM32L0xx")


#Application
if(APPLICATION STREQUAL AT)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave)

elseif(APPLICATION STREQUAL End_Node)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node)

elseif(APPLICATION STREQUAL PingPong) 
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong)  
endif()

    