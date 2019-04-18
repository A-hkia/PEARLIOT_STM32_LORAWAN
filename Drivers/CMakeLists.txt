##-------------Drivers choice and configuration-------------##
#
#
project(drivers)
cmake_minimum_required(VERSION 3.6)

#Subdirectories
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/BSP)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/CMSIS)

#Series
if(SERIES STREQUAL STM32L0xx)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/STM32L0xx_HAL_Driver)
elseif(SERIES STREQUAL STM32L1xx)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/STM32L1xx_HAL_Driver)
elseif(SERIES STREQUAL STM32L4xx)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/STM32L4xx_HAL_Driver)
endif()


