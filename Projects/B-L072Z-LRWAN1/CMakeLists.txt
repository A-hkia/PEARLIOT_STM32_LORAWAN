##-------------Application choice for BL072Z-------------##
#
#
project(B-L072Z-LRWAN1)
cmake_minimum_required(VERSION 3.6)

#Application
if(APPLICATION STREQUAL AT_Slave)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/AT_Slave)

elseif(APPLICATION STREQUAL End_Node)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/End_Node)

elseif(APPLICATION STREQUAL PingPong) 
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Applications/LoRa/PingPong)  
endif()

    