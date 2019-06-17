##-------------First CMakeLists.txt of the project-------------##
#
#
project(stm_lora)
cmake_minimum_required(VERSION 3.6)

enable_language(ASM)

#Listing the subdirectories used in the cmake files
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Drivers)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Middlewares)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Projects)

# Allow switching of Projects
set(PROJECT_LIST B-L072Z-LRWAN1 MIROMICO STM32L053R8-Nucleo STM32L073RZ-Nucleo STM32L152RE-Nucleo STM32L476RG-Nucleo STM32WB55-Nucleo)
set(PROJECT B-L072Z-LRWAN1 CACHE STRING "Default Project is B-L072Z-LRWAN1")
set_property(CACHE PROJECT PROPERTY STRINGS ${PROJECT_LIST})


#---------------------------------------------------------------------------------------
# Application Options
#---------------------------------------------------------------------------------------

# Allow switching of Applications
set(APPLICATION_LIST AT End_Node PingPong)
set(APPLICATION End_Node CACHE STRING "Default Application is End Node")
set_property(CACHE APPLICATION PROPERTY STRINGS ${APPLICATION_LIST})

# Allow switching of Radio Modules
set(RADIO_LIST sx126x sx1272 sx1276)
set(RADIO sx1276 CACHE STRING "Default Radio Module is sx1276")
set_property(CACHE RADIO PROPERTY STRINGS ${RADIO_LIST})

# Allow switching of Sensor Boards
set(SENSOR_BOARD_LIST X_NUCLEO_IKS01A1 X_NUCLEO_IKS01A2 NO_SENSOR_BOARD)
set(SENSOR_BOARD X_NUCLEO_IKS01A2 CACHE STRING "Default Sensor Board is X_NUCLEO_IKS01A2")
set_property(CACHE SENSOR_BOARD PROPERTY STRINGS ${SENSOR_BOARD_LIST})

# Allow switching of Cryptography methods
set(CRYPTO_LIST PearlIoT Soft-se)
set(CRYPTO PearlIoT CACHE STRING "Default Crypto is PearlIoT")
set_property(CACHE CRYPTO PROPERTY STRINGS ${CRYPTO_LIST})

# Allow switching of Series - No cache is used since the serie is chosen automatically by the project choice
set(SERIES_LIST STM32L0xx STM32L1xx STM32L4xx STM32WBxx)
