##-------------Miromico_drivers-------------##
#
#
project(Miromico_drivers)
cmake_minimum_required(VERSION 3.6)

#Specify the project's source files
file(GLOB ${PROJECT_NAME}_SOURCES
    #"${CMAKE_CURRENT_SOURCE_DIR}/Common/*c"   
#    "${CMAKE_CURRENT_SOURCE_DIR}/Components/common/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/Components/mx25/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/Components/sts_sht_21/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/Components/sx1272/*.c"   
    "${CMAKE_CURRENT_SOURCE_DIR}/sx1272fmlr/*c"   

)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})


#Add all the header files
target_include_directories( ${PROJECT_NAME} PUBLIC
  #  ${CMAKE_CURRENT_SOURCE_DIR}
#    ${CMAKE_CURRENT_SOURCE_DIR}/Common
#    ${CMAKE_CURRENT_SOURCE_DIR}/Components/common
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/mx25
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/sts_sht_21
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/sx1272
    ${CMAKE_CURRENT_SOURCE_DIR}/sx1272fmlr
    $<TARGET_PROPERTY:middlewares,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:Components,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:${PROJECT}-${APPLICATION},INTERFACE_INCLUDE_DIRECTORIES>
)

#Specify the C standard whose features are requested to build this target
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
