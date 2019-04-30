##-------------Application choice for STM32L152RE-Nucleo-------------##
#
#
project(STM32L152RE-Nucleo)
cmake_minimum_required(VERSION 3.6)

set(SERIES STM32L1xx CACHE STRING "Serie chosen is STM32L1xx")


#Application
if(APPLICATION STREQUAL AT)
	message(FATAL_ERROR "You cannot choose AT mode with STM32L152RE-Nucleo")

elseif(APPLICATION STREQUAL End_Node)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node)

elseif(APPLICATION STREQUAL PingPong) 
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong)  
endif()

    