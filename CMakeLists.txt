set(MCU_FAMILY STM32F4xx)
set(CMAKE_TOOLCHAIN_FILE STM32Toolchain.cmake)
cmake_minimum_required(VERSION 3.6)

project(chassis C CXX ASM)

add_definitions(-DSTM32F427xx)
add_definitions(-DUSE_HAL_LIBRARY)
add_definitions(-Wall -ggdb)
add_definitions(-DARM_MATH_CM4)

set(COMMON_FLAGS "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -lc -specs=nosys.specs -mthumb -mthumb-interwork -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")

set(CMAKE_OBJCOPY arm-none-eabi-objcopy)

set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -mcpu=cortex-m4 -mthumb -T ${CMAKE_SOURCE_DIR}/STM32F427IIHx_FLASH.ld")

set(USER_INCLUDE Inc)
set(BSP_INCLUDE bsp/include)
set(MAVLINK_INCLUDE mavlink)
set(CMSIS_DEVICE_INCLUDE Drivers/CMSIS/Device/ST/STM32F4xx/Include)
set(CMSIS_INCLUDE Drivers/CMSIS/Include)
set(HAL_INCLUDE Drivers/STM32F4xx_HAL_Driver/Inc)
set(USBD_INCLUDE Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc)
set(RTOS_INCLUDE Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS)

file(GLOB_RECURSE USER_INCLUDE_F ${USER_INCLUDE}/*.h)
file(GLOB_RECURSE BSP_INCLUDE_F ${BSP_INCLUDE}/**.h
     ${BSP_INCLUDE}/**.hpp)
file(GLOB_RECURSE MAVLINK_INCLUDE_F ${MAVLINK_INCLUDE}/**.h)
file(GLOB_RECURSE CMSIS_DEVICE_INCLUDE_F ${CMSIS_DEVICE_INCLUDE}/*.h)
file(GLOB_RECURSE CMSIS_INCLUDE_F ${CMSIS_INCLUDE}/*.h)
file(GLOB_RECURSE HAL_INCLUDE_F ${HAL_INCLUDE}/*.h)
file(GLOB_RECURSE USBD_INCLUDE_F Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/*.h
     Middlewares/ST/STM32_USB_Device_Library/Core/Inc/*.h)
file(GLOB_RECURSE RTOS_INCLUDE_F ${RTOS_INCLUDE}/*.h
     Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/*.h
     Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/*.h)

file(GLOB_RECURSE USER_SOURCES Src/*.c)
file(GLOB_RECURSE BSP_SOURCES bsp/src/*.c
     bsp/src/*.cpp)
file(GLOB_RECURSE HAL_SOURCES Drivers/STM32F4xx_HAL_DRIVER/Src/*.c)
file(GLOB_RECURSE CMSIS_SYSTEM Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_STM32F4xx.c)
file(GLOB_RECURSE CMSIS_STARTUP startup/startup_stm32f427xx.s)
file(GLOB_RECURSE USBD_SOURCES Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c
        Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c)
file(GLOB_RECURSE RTOS_SOURCES Middlewares/Third_Party/FreeRTOS/Source/*.c
     Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/*.c
     Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/*.c
     Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/*.c)

set(SOURCE_FILES ${USER_SOURCES} ${BSP_SOURCES} ${RTOS_SOURCES} ${USBD_SOURCES} ${HAL_SOURCES} ${CMSIS_SYSTEM} ${CMSIS_STARTUP} STM32F427IIHx_FLASH.ld
                 ${USER_INCLUDE_F} ${BSP_INCLUDE_F} ${MAVLINK_INCLUDE_F} ${USBD_INCLUDE_F} ${CMSIS_DEVICE_INCLUDE_F} ${CMSIS_INCLUDE_F} ${HAL_INCLUDE_F})

include_directories(${USER_INCLUDE} ${BSP_INCLUDE} ${MAVLINK_INCLUDE} ${CMSIS_DEVICE_INCLUDE} ${CMSIS_INCLUDE} ${USBD_INCLUDE} ${HAL_INCLUDE} ${RTOS_INCLUDE})

# Rust executable
add_custom_command(
        OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/target/thumbv7em-none-eabihf/debug/librust32.a
        COMMAND
        CARGO_TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR}/target xargo build --target thumbv7em-none-eabihf
        WORKING_DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/rust
        VERBATIM
)

add_custom_target(rust_target DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/target/thumbv7em-none-eabihf/debug/librust32.a)

# create an library target out of the library compilation result
add_library(demo STATIC IMPORTED GLOBAL)
add_dependencies(demo rust_target)

# specify where the library is and where to find the headers
set_target_properties(demo
        PROPERTIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/target/thumbv7em-none-eabihf/debug/librust32.a)

# END Rust
add_executable(${PROJECT_NAME}.elf ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(${CMAKE_PROJECT_NAME}.elf demo)

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