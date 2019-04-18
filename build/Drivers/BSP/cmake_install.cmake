# Install script for directory: /home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/build/Drivers/BSP/B-L072Z-LRWAN1/cmake_install.cmake")
  include("/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/build/Drivers/BSP/CMWX1ZZABZ-0xx/cmake_install.cmake")
  include("/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/build/Drivers/BSP/Components/cmake_install.cmake")
  include("/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/build/Drivers/BSP/X_NUCLEO_IKS01A1/cmake_install.cmake")

endif()

