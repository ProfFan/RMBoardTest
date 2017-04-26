include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc GCC)
CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GCC)
set(CMAKE_ASM_COMPILER arm-none-eabi-as)

set(CMAKE_OBJCOPY arm-none-eabi-objcopy)

set(COMMON_FLAGS "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -lc -specs=nosys.specs -mthumb -mthumb-interwork -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")

set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T ${CMAKE_SOURCE_DIR}/STM32F427IIHx_FLASH.ld")
