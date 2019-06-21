## STM32L0 target specific CMake file
##

if(NOT DEFINED LINKER_SCRIPT)
message(FATAL_ERROR "No linker script defined")
endif(NOT DEFINED LINKER_SCRIPT)
#message("Linker script: ${LINKER_SCRIPT}")

#---------------------------------------------------------------------------------------
# Set compiler/linker flags
#---------------------------------------------------------------------------------------

# Object build options
set(OBJECT_GEN_FLAGS "-Og -g -mthumb -g2 -fno-builtin -mcpu=cortex-m0plus -Wall -Wextra -pedantic -DUSE_LRWAN_NS1 -DUSE_SX126X_DVK -DSTM32WB55xx -DUSE_STM32WB55_NUCLEO -Wno-unused-parameter -DUSE_BAND_868 -DUSE_MODEM_LORA -DUSE_MDM32L07X01 -DUSE_HAL_DRIVER -DDEBUG -DLOW_POWER_DISABLE -DREGION_EU868 -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize")

set(CMAKE_C_FLAGS "${OBJECT_GEN_FLAGS} -std=gnu99 " CACHE INTERNAL "C Compiler options")
set(CMAKE_CXX_FLAGS "${OBJECT_GEN_FLAGS} -std=c++11 " CACHE INTERNAL "C++ Compiler options")
set(CMAKE_ASM_FLAGS "${OBJECT_GEN_FLAGS} -x assembler-with-cpp " CACHE INTERNAL "ASM Compiler options")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -mthumb -g2 -mcpu=cortex-m0plus -mabi=aapcs -T${LINKER_SCRIPT} -Wl,-Map=${CMAKE_PROJECT_NAME}.map" CACHE INTERNAL "Linker options")