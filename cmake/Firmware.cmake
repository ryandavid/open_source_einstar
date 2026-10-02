# The scanner's FX3 firmware (firmware/), built as part of this build when EINSTAR_BUILD_FIRMWARE is ON.
# It cross-compiles for the FX3's ARM9 with the Arm GNU Toolchain, and a CMake project has one compiler per
# language, so firmware/ stays its own project and this drives it as an external project: it is configured
# and built by `cmake --build` here, into <build>/firmware/<lang>/ (einstar_fx3.{elf,img} + update package).
include(ExternalProject)

set(EINSTAR_FIRMWARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/firmware")

set(FX3_ARMGNU_GCC "/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi/bin/arm-none-eabi-gcc"
    CACHE FILEPATH "Arm GNU Toolchain arm-none-eabi-gcc (firmware)")
if(NOT EXISTS "${FX3_ARMGNU_GCC}")
  message(FATAL_ERROR "EINSTAR_BUILD_FIRMWARE: no Arm GNU Toolchain gcc at ${FX3_ARMGNU_GCC}; install it or set FX3_ARMGNU_GCC")
endif()
foreach(input "sdk/inc" "sdk/lib" "sdk/fw_build" "fpga/EinScan10_01_FPGA_V3.7.bin")
  if(NOT EXISTS "${EINSTAR_FIRMWARE_DIR}/${input}")
    message(FATAL_ERROR "EINSTAR_BUILD_FIRMWARE: firmware/${input} is missing. The FX3 SDK and the FPGA bitstream "
                        "are user-supplied; see \"Inputs you must supply\" in firmware/README.md")
  endif()
endforeach()

# The firmware project's own settings, passed through when set here (-DFX3_OPT=-O2, -DFW_LANGUAGES=EN;CH, ...)
set(EINSTAR_FIRMWARE_ARGS "-DFX3_ARMGNU_GCC:FILEPATH=${FX3_ARMGNU_GCC}")
foreach(var FX3_HOST_CC FX3_OPT FW_LANGUAGES)
  if(DEFINED ${var})
    string(REPLACE ";" "\\;" value "${${var}}")
    list(APPEND EINSTAR_FIRMWARE_ARGS "-D${var}:STRING=${value}")
  endif()
endforeach()

ExternalProject_Add(firmware
  SOURCE_DIR "${EINSTAR_FIRMWARE_DIR}"
  BINARY_DIR "${CMAKE_BINARY_DIR}/firmware"
  CMAKE_CACHE_ARGS ${EINSTAR_FIRMWARE_ARGS}
  CONFIGURE_HANDLED_BY_BUILD ON
  BUILD_ALWAYS ON
  INSTALL_COMMAND ""
  USES_TERMINAL_BUILD ON)
