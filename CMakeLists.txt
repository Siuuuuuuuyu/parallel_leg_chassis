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
set(CMAKE_PROJECT_NAME parallel_leg_chassis)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Bsp/Inc/bsp_can.h
        Bsp/Src/bsp_can.c
        Bsp/Inc/bsp_uart.h
        Bsp/Src/bsp_uart.c
        IMU/Inc/BMI088reg.h
        IMU/Inc/BMI088middleware.h
        IMU/Inc/BMI088driver.h
        IMU/Src/BMI088driver.c
        IMU/Src/BMI088middleware.c
        Bsp/Inc/bsp_dwt.h
        Bsp/Src/bsp_dwt.c
        Device/Inc/dm_motor_ctrl.h
        Device/Src/dm_motor_ctrl.c
        Algorithm/Inc/KalmanFilter.h
        Algorithm/Inc/QuaternionEKF.h
        Algorithm/Inc/MahonyAHRS.h
        Algorithm/Src/KalmanFilter.c
        Algorithm/Src/MahonyAHRS.c
        Algorithm/Src/QuaternionEKF.c
        PID/Inc/pid.h
        PID/Src/pid.c
        Application/Inc/imu_task.h
        Application/Src/imu_task.c
        Device/Inc/dji_motor_ctrl.h
        Device/Src/dji_motor_ctrl.c
        Device/Inc/cm_device.h
        Device/Src/cm_device.c
        Device/Inc/motor_device.h
        Device/Src/motor_device.c
        Algorithm/Inc/LegController.h
        Algorithm/Src/LegController.c
        Algorithm/Inc/StateObserver.h
        Algorithm/Src/StateObserver.c
        Bsp/Inc/bsp_pwm.h
        Bsp/Src/bsp_pwm.c
        Application/Inc/chassis_task.h
        Application/Src/chassis_task.c
        Application/Inc/observer_task.h
        Application/Src/observer_task.c
        Application/Inc/chassis_ctrl.h
        Application/Src/chassis_ctrl.c
        Device/Inc/remote_ctrl.h
        Device/Src/remote_ctrl.c
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
        Bsp/Inc
        Bsp/Src
        Application/Inc
        Application/Src
        Algorithm/Inc
        Algorithm/Src
        IMU/Inc
        IMU/Src
        PID/Inc
        PID/Src
        Device/Inc
        Device/Src
        ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
        ARM_MATH_CM7
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

set(CMSIS_DSP_LIB_PATH "${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/DSP/Lib/GCC")
if(EXISTS "${CMSIS_DSP_LIB_PATH}/libarm_cortexM7lfdp_math.a")
    set(CMSIS_DSP_LIB "${CMSIS_DSP_LIB_PATH}/libarm_cortexM7lfdp_math.a")
elseif(EXISTS "${CMSIS_DSP_LIB_PATH}/libarm_cortexM7lfsp_math.a")
    set(CMSIS_DSP_LIB "${CMSIS_DSP_LIB_PATH}/libarm_cortexM7lfsp_math.a")
else()
    message(FATAL_ERROR "未找到CMSIS-DSP库，请确保路径正确: ${CMSIS_DSP_LIB_PATH}")
endif()

message(STATUS "使用CMSIS-DSP库: ${CMSIS_DSP_LIB}")

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
        ${CMSIS_DSP_LIB}
)
