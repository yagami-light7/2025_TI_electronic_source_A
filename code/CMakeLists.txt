cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Pro_TI_electronic_source)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        Users/0-APL/APL_CallBack/CallBack_ADC/CallBack_ADC.cpp
        Users/0-APL/APL_CallBack/CallBack_TIM/CallBack_TIM.cpp
        Users/0-APL/APL_CallBack/CallBack_HRTIM/CallBack_HRTIM.cpp
        Users/0-APL/APL_USART/APL_UASRT.cpp
        Users/0-APL/Thread_Adjust/Thread_Adjust.cpp

        Users/1-MWL/MWL_Transform/MWL_Transform.cpp
        Users/1-MWL/MWL_Data_Utils/MWL_Data_Utils.cpp
        Users/1-MWL/MWL_PR/MWL_PR.cpp
        Users/1-MWL/MWL_PID/MWL_PID.cpp
        Users/1-MWL/MWL_PLL/MWL_PLL.cpp
        Users/1-MWL/MWL_Toolbox/MWL_Toolbox.cpp

        Users/2-HDL/HDL_AC_Measure/HDL_AC_Measure.cpp
        Users/2-HDL/HDL_PWM_Modulation/HDL_PWM_Modulation.cpp
        Users/2-HDL/HDL_bearADC.Measure/HDL_bearADC_Measure.cpp
        Users/2-HDL/HDL_PLL_Function/HDL_PLL_Function.cpp
        Users/2-HDL/HDL_UART_Screen/HDL_UART_Screen.cpp

        Users/3-HAL/HAL_HUB/HAL_HUB.cpp
        Users/3-HAL/HAL_ADC/HAL_ADC.cpp
        Users/3-HAL/HAL_HRTIM/HAL_HRTIM.cpp
        Users/3-HAL/HAL_TIM/HAL_TIM.cpp
        Users/3-HAL/HAl_SPI/HAL_SPI.cpp
        Users/3-HAL/HAL_USART/HAL_USART.cpp
        Users/3-HAL/HAL_UART_Screen/HAL_UART_Screen.cpp

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Users/0-APL/APL_CallBack/CallBack_ADC
        Users/0-APL/APL_CallBack/CallBack_TIM
        Users/0-APL/APL_CallBack/CallBack_HRTIM
        Users/0-APL/APL_USART
        Users/0-APL/Thread_Adjust

        Users/1-MWL/MWL_Transform
        Users/1-MWL/MWL_Data_Utils
        Users/1-MWL/MWL_PR
        Users/1-MWL/MWL_PID
        Users/1-MWL/MWL_PLL
        Users/1-MWL/MWL_Toolbox

        Users/2-HDL/HDL_AC_Measure
        Users/2-HDL/HDL_bearADC.Measure
        Users/2-HDL/HDL_PWM_Modulation
        Users/2-HDL/HDL_PLL_Function
        Users/2-HDL/HDL_UART_Screen

        Users/3-HAL/HAL_ADC
        Users/3-HAL/HAL_HUB
        Users/3-HAL/HAL_HRTIM
        Users/3-HAL/HAL_TIM
        Users/3-HAL/HAl_SPI
        Users/3-HAL/HAL_USART
        Users/3-HAL/HAL_UART_Screen
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
        ARM_MATH_CM4,__FPU_USED=1U,__FPU_PRESENT=1U,__CC_ARM,ARM_MATH_MATR*IX_CHECK,ARM_MATH_ROUNDING
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
