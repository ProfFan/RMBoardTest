set(MCU_FAMILY STM32F4xx)
set(CMAKE_TOOLCHAIN_FILE STM32Toolchain.cmake)
cmake_minimum_required(VERSION 3.6)

project(chassis C ASM)

add_definitions(-DSTM32F427xx)
add_definitions(-DUSE_HAL_LIBRARY)
add_definitions(-Wall -ggdb)
add_definitions(-DARM_MATH_CM4)

set(COMMON_FLAGS "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -lc -specs=nosys.specs -mthumb -mthumb-interwork -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")

set(CMAKE_OBJCOPY arm-none-eabi-objcopy)

set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m4 -mthumb -T ${CMAKE_SOURCE_DIR}/STM32F427IIHx_FLASH.ld")

set(USER_INCLUDE Inc)
set(CMSIS_DEVICE_INCLUDE Drivers/CMSIS/Device/ST/STM32F4xx/Include)
set(CMSIS_INCLUDE Drivers/CMSIS/Include)
set(HAL_INCLUDE Drivers/STM32F4xx_HAL_Driver/Inc)
set(RTOS_INCLUDE Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS)

file(GLOB_RECURSE USER_INCLUDE_F ${USER_INCLUDE}/*.h)
file(GLOB_RECURSE CMSIS_DEVICE_INCLUDE_F ${CMSIS_DEVICE_INCLUDE}/*.h)
file(GLOB_RECURSE CMSIS_INCLUDE_F ${CMSIS_INCLUDE}/*.h)
file(GLOB_RECURSE HAL_INCLUDE_F ${HAL_INCLUDE}/*.h)
file(GLOB_RECURSE RTOS_INCLUDE_F ${RTOS_INCLUDE}/*.h
     Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/*.h
     Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/*.h)

file(GLOB_RECURSE USER_SOURCES Src/*.c)
file(GLOB_RECURSE HAL_SOURCES Drivers/STM32F4xx_HAL_DRIVER/Src/*.c)
file(GLOB_RECURSE CMSIS_SYSTEM Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_STM32F4xx.c)
file(GLOB_RECURSE CMSIS_STARTUP startup/startup_stm32f427xx.s)
file(GLOB_RECURSE RTOS_SOURCES Middlewares/Third_Party/FreeRTOS/Source/*.c
     Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/*.c
     Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/*.c
     Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/*.c)

set(SOURCE_FILES ${USER_SOURCES} ${RTOS_SOURCES} ${HAL_SOURCES} ${CMSIS_SYSTEM} ${CMSIS_STARTUP} STM32F427IIHx_FLASH.ld
                 ${USER_INCLUDE_F} ${CMSIS_DEVICE_INCLUDE_F} ${CMSIS_INCLUDE_F} ${HAL_INCLUDE_F})

include_directories(${USER_INCLUDE} ${CMSIS_DEVICE_INCLUDE} ${CMSIS_INCLUDE} ${HAL_INCLUDE} ${RTOS_INCLUDE})

add_executable(${PROJECT_NAME}.elf ${SOURCE_FILES})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.map")
set(HEX_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/build
        COMMENT "Creating build directory")
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} 
Building ${BIN_FILE}")

add_custom_target(flash
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMAND openocd -f openocd_flash.cfg
        COMMENT "Flashing the target processor..."
        DEPENDS ${PROJECT_NAME}.elf)
add_custom_command(TARGET flash POST_BUILD COMMENT "Flashing finished!")

add_custom_target(debug
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMAND openocd -f openocd_debug.cfg
        COMMENT "Starting debug session..."
        DEPENDS ${PROJECT_NAME}.elf)