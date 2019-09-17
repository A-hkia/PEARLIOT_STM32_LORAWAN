# PEARLIOT_STM32_LORAWAN
This project is the result of the integration of IDEMIA's PEARL IOT secure element to STM32 microcontrollers with the LoRaWAN protocol.
It is an extension to the I-CUBE-LRWAN project ( https://www.st.com/en/embedded-software/i-cube-lrwan.html ).
The latter being the LoRa software expansion for STM32Cube; a set of embedded software bricks designed to ease development for STM32 platforms.

##Description
PEARL IOT is a secure element developed by IDEMIA, it secures the communication of the microcontroller on which it is attached.
It is paired with a remote secure server, both of which are used to store cryptographic keys.
This project contains software of stacks of the PEARL IOT, LoRaWAN protocol and STM32 microcontrollers.
PEARL_IOT_STM32_LORAWAN was developed to ease the integration of microcontrollers with the PEARL IOT.
The 'I-CUBE-LRWAN-Cmake.pdf' file features a tutorial on how to integrate a new STM32 microcontroller to the project. 

##Specifications
PEARLIOT_STM32_LORAWAN features source code of LoRa applications for the following boards:
    • B-L072Z-LRWAN1
    • STM32L053R8-Nucleo
    • STM32L073RZ-Nucleo
    • STM32L152RE-Nucleo
    • STM32L476RG-Nucleo
    • FMLR_61_X_RSS3
    • P-Nucleo-WB55
    
It supports the usage of the following electronic components:
    • Radio module:
        o SX126x
        o SX1272
        o SX1276
    • Sensor board:
        o X_NUCLEO_IKS01A1
        o X_NUCLEO_IKS01A2
        
This project uses the 1.1.1 LoRaWAN version.

##Installation:
In order to run the project on linux environment, cmake must be installed as well as the Arm development environmet for eclipse which include: node-js, xpm. To install the arm compiler:
xpm install --global @gnu-mcu-eclipse/arm-none-eabi-gcc 
xpm install --global @gnu-mcu-eclipse/openocd

The cache, located in the CmakeCache.txt in the build directory, should be cleared when importing the project from another PC.

On eclipse IDE, the project must be converted to a C++ project, with the ARM file GCC option. In the build options (Properties/Builds/C++), the build’s directory should be specified and the build variable should be set to ‘make’.

##Usage
The expected output of this project is a successful Join Request and Uplink sent to a LoRa server.
Sample usage:
Create a 'build' folder in your directory and run the ccmake command to choose the configurations.
The next steps are configuring ('c' command), generating ('g' command) and running the cmake command.

N.B.: To run the cmake command, we must specify the location of the installed toolcahin, Example:
      cmake -DCMAKE_TOOLCHAIN_FILE="cmake/toolchain-arm-none-eabi.cmake" -DTOOLCHAIN_PREFIX="/home/abdallah/opt/xPacks/@gnu-mcu-eclipse/arm-none-eabi-gcc/8.2.1-1.4.1/.content/" ..
The executable is then created and run on the microcontroller.

##Authors
This project was developed by Abdallah ABDELHAMID ( https://www.linkedin.com/in/abdallah-abdelhamid/ ) along with Mourad BENMEHIRISSE under the supervision of François LORRAIN ( https://www.linkedin.com/in/francois-lorrain-8b7588/ ).

##Licence
MIT License

Copyright (c) 2019 Abdallah ABDELHAMID

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files PEARLIOT_STM32_LORAWAN, to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

