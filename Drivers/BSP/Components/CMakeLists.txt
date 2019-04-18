##-------------Components choice and configuration-------------##
#
#
project(Components)
cmake_minimum_required(VERSION 3.6)

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/hts221/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/lis3mdl/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/lps22hb/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/lps25hb/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/lsm303agr/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/lsm6ds0/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/lsm6ds3/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/lsm6dsl/*.c"   
    
)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})


target_include_directories( ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Common   
    ${CMAKE_CURRENT_SOURCE_DIR}/hts221   
    ${CMAKE_CURRENT_SOURCE_DIR}/lis3mdl  
    ${CMAKE_CURRENT_SOURCE_DIR}/lps22hb  
    ${CMAKE_CURRENT_SOURCE_DIR}/lps25hb  
    ${CMAKE_CURRENT_SOURCE_DIR}/lsm303agr 
    ${CMAKE_CURRENT_SOURCE_DIR}/lsm6ds0 
    ${CMAKE_CURRENT_SOURCE_DIR}/lsm6ds3 
    ${CMAKE_CURRENT_SOURCE_DIR}/lsm6dsl   
    $<TARGET_PROPERTY:middlewares,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:B-L072Z-LRWAN1-End_Node,INTERFACE_INCLUDE_DIRECTORIES>
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)

#Radio
if(RADIO STREQUAL sx126x)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sx126x)
	
elseif(RADIO STREQUAL sx1272)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sx1272)

elseif(RADIO STREQUAL sx1276)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sx1276)
	
endif()


