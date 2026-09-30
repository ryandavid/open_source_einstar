# Toolchain: the Arm GNU Toolchain's gcc, its own newlib and
# libgcc (the multilib -mcpu=arm926ej-s -marm selects: ARM mode, soft float).
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(FX3_ARMGNU_GCC "/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi/bin/arm-none-eabi-gcc"
    CACHE FILEPATH "Arm GNU Toolchain arm-none-eabi-gcc")
set(FX3_HOST_CC "cc" CACHE STRING "host C compiler for the build tools (elf2img, mkpackage)")

set(CMAKE_C_COMPILER "${FX3_ARMGNU_GCC}")
set(CMAKE_ASM_COMPILER "${FX3_ARMGNU_GCC}")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
