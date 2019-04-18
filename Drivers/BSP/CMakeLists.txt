##-------------BSP-------------##
#
#
project(stm-lora)
cmake_minimum_required(VERSION 3.6)

#Project
if(PROJECT STREQUAL B-L072Z-LRWAN1)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/B-L072Z-LRWAN1)
	#Module Murata
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/CMWX1ZZABZ-0xx)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Components)
	
endif()

#Sensor Board
if(SENSOR_BOARD STREQUAL X_NUCLEO_IKS01A1) 
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/X_NUCLEO_IKS01A1)
elseif(SENSOR_BOARD STREQUAL X_NUCLEO_IKS01A2)	
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/X_NUCLEO_IKS01A2)
endif()