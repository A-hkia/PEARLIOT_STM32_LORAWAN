##-------------Project choice and configuration-------------##
#
#
project(CMSIS)
cmake_minimum_required(VERSION 3.6)

#Series
if(SERIES STREQUAL STM32L0xx)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Device/ST/STM32L0xx)
elseif(SERIES STREQUAL STM32L1xx)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Device/ST/STM32L1xx)
elseif(SERIES STREQUAL STM32L4xx)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Device/ST/STM32L4xx)
elseif(SERIES STREQUAL STM32WBxx)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Device/ST/STM32WBxx)
endif()

