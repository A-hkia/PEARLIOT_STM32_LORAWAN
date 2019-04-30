# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# compile C with /home/abdallah/opt/xPacks/@gnu-mcu-eclipse/arm-none-eabi-gcc/8.2.1-1.4.1/.content//bin/arm-none-eabi-gcc
C_FLAGS = -Og -g -mthumb -g2 -fno-builtin -mcpu=cortex-m0plus -Wall -Wextra -pedantic -DUSE_LRWAN_NS1 -Wno-unused-parameter -DUSE_BAND_868 -DUSE_MODEM_LORA -DUSE_MDM32L07X01 -DUSE_HAL_DRIVER -DSTM32L072xx -DDEBUG -DLOW_POWER_DISABLE -DUSE_B_L072Z_LRWAN1 -DREGION_EU868 -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize -std=gnu99    -std=gnu11

C_DEFINES = 

C_INCLUDES = -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/Components/Common -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/Components/hts221 -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/Components/lis3mdl -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/Components/lps22hb -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/Components/lps25hb -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/Components/lsm303agr -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/Components/lsm6ds0 -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/Components/lsm6ds3 -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/Components/lsm6dsl -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Middlewares/Third_Party/LoRaWAN/Conf -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Middlewares/Third_Party/LoRaWAN/Conf/Inc -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Middlewares/Third_Party/LoRaWAN/Core -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Middlewares/Third_Party/LoRaWAN/Crypto -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Middlewares/Third_Party/LoRaWAN/Mac -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Middlewares/Third_Party/LoRaWAN/Mac/region -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Middlewares/Third_Party/LoRaWAN/Phy -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Middlewares/Third_Party/LoRaWAN/Utilities -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Projects/B-L072Z-LRWAN1/Applications/LoRa/End_Node/Core/inc -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Projects/B-L072Z-LRWAN1/Applications/LoRa/End_Node/LoRaWAN/App/inc -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/B-L072Z-LRWAN1 -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/CMWX1ZZABZ-0xx -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/CMSIS/Device/ST/STM32L0xx/Include -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/CMSIS/Device/ST/STM32L0xx/../../../Include -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/CMSIS/Device/ST/STM32L0xx/../../../RTOS/Template -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/CMSIS/Device/ST/STM32L0xx/../../../Documentation/Core/html -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/CMSIS/Device/ST/STM32L0xx/../../../Documentation/Core/html/search -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/CMSIS/Device/ST/STM32L0xx/../../../Documentation/DSP/html -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/CMSIS/Device/ST/STM32L0xx/../../../Documentation/DSP/html/search -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/CMSIS/Device/ST/STM32L0xx/../../../Documentation/General/html -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/CMSIS/Device/ST/STM32L0xx/../../../Lib/ARM -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/CMSIS/Device/ST/STM32L0xx/../../../Lib/GCC -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/STM32L0xx_HAL_Driver/Inc -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/X_NUCLEO_IKS01A2 -I/home/abdallah/eclipse-workspace/STM32CubeExpansion_LRWAN_V1.2.0/Drivers/BSP/Components/sx1276 

