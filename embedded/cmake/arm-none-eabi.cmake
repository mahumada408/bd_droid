# Cross toolchain: Arm GNU Toolchain (arm-none-eabi), Cortex-M7 with double-precision FPU.
# ARM_TOOLCHAIN_DIR defaults to ~/.local/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(NOT ARM_TOOLCHAIN_DIR)
  if(DEFINED ENV{ARM_TOOLCHAIN_DIR})
    set(ARM_TOOLCHAIN_DIR $ENV{ARM_TOOLCHAIN_DIR})
  else()
    set(ARM_TOOLCHAIN_DIR "$ENV{HOME}/.local/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi")
  endif()
endif()
set(ARM_TOOLCHAIN_DIR "${ARM_TOOLCHAIN_DIR}" CACHE PATH "Arm GNU toolchain root")

set(CMAKE_C_COMPILER   ${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER ${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc)
set(CMAKE_OBJCOPY      ${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-objcopy CACHE FILEPATH "")
set(CMAKE_SIZE         ${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-size CACHE FILEPATH "")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(MCU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT   "${MCU_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT "${MCU_FLAGS} -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS_INIT "${MCU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${MCU_FLAGS} -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
