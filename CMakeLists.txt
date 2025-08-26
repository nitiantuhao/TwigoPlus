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
set(CMAKE_PROJECT_NAME Twigo)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        App/pin_definitions.h
        App/Sensor/dmpKey.h
        App/Sensor/dmpmap.h
        App/Sensor/inv_mpu.c
        App/Sensor/inv_mpu.h
        App/Sensor/inv_mpu_dmp_motion_driver.c
        App/Sensor/inv_mpu_dmp_motion_driver.h
        App/Sensor/mpu6050_dmp.c
        App/Sensor/mpu6050_dmp.h
        App/Motor/tb6612.c
        App/Motor/tb6612.h
        App/Comm/bluetooth_debug.c
        App/Comm/bluetooth_debug.h
        App/Comm/font.c
        App/Comm/font.h
        App/Comm/hc05.c
        App/Comm/hc05.h
        App/Comm/oled.c
        App/Comm/oled.h
        App/Comm/oled_debug.c
        App/Comm/oled_debug.h
        App/Balance/balance_control.c
        App/Balance/balance_control.h
        App/Balance/encoder.c
        App/Balance/encoder.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        ${CMAKE_CURRENT_SOURCE_DIR}/App
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u _printf_float
)