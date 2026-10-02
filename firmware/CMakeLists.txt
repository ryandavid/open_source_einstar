# Einstar FX3 firmware (open build). Reconstructed application built with the Arm GNU Toolchain against
# the Cypress FX3 SDK. Not byte-identical to the vendor's image; verified behaviourally equivalent to it
# with the differential emulator (tools/fx3emu in the repository):
#   python -m fx3emu diff        # (uses build/firmware/EN/einstar_fx3.elf)
#
# Built from the top-level project, which drives this one as an external project (cmake/Firmware.cmake):
#   cmake -B build -DEINSTAR_BUILD_FIRMWARE=ON && cmake --build build   # -> build/firmware/EN/einstar_fx3.{elf,img} + package
#   ./check.sh                                              # build + differential vs the vendor image
#
# Everything the build needs is in this tree (sdk/, fpga/) except the toolchain (cmake/fx3-toolchain.cmake).
cmake_minimum_required(VERSION 3.25)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/fx3-toolchain.cmake" CACHE FILEPATH "")
endif()
project(einstar_fx3 C ASM)

set(FX3_OPT "-O0" CACHE STRING "optimisation level (-O0 matches the vendor's shipping build; higher is verified equivalent by tools/fx3emu but is not the default)")
set(FW_LANGUAGES EN CACHE STRING "firmware language variants to build (EN and/or CH)")

# The SDK build flags are the only compile flags: no CMake build-type flags.
foreach(cfg DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set(CMAKE_C_FLAGS_${cfg} "")
    set(CMAKE_ASM_FLAGS_${cfg} "")
endforeach()

set(SDK "${CMAKE_CURRENT_SOURCE_DIR}/sdk")
set(FPGA_BITSTREAM "${CMAKE_CURRENT_SOURCE_DIR}/fpga/EinScan10_01_FPGA_V3.7.bin")
set(PACKAGE_NAME "EinScan10_01_SC130_OPN_V2.10_FPGA_V3.7_%LANG%_IAP.img")   # (version.c: OPN marks this build)

execute_process(COMMAND "${CMAKE_C_COMPILER}" --version OUTPUT_VARIABLE _ccv)
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" _ccver "${_ccv}")
message(STATUS "FX3 firmware: gcc ${_ccver} ${FX3_OPT}, languages: ${FW_LANGUAGES}")

# ---- host tools (elf2img, mkpackage) built with the host compiler
set(HOST_BIN "${CMAKE_BINARY_DIR}/host")
file(MAKE_DIRECTORY "${HOST_BIN}")
add_custom_command(OUTPUT "${HOST_BIN}/elf2img"
    COMMAND ${FX3_HOST_CC} -O2 -w -o "${HOST_BIN}/elf2img" "${SDK}/util/elf2img/elf2img.c"
    DEPENDS "${SDK}/util/elf2img/elf2img.c" COMMENT "Building host tool elf2img" VERBATIM)
add_custom_command(OUTPUT "${HOST_BIN}/mkpackage"
    COMMAND ${FX3_HOST_CC} -O2 -Wall -o "${HOST_BIN}/mkpackage" "${CMAKE_CURRENT_SOURCE_DIR}/tools/mkpackage.c"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tools/mkpackage.c" COMMENT "Building host tool mkpackage" VERBATIM)

# ---- compile flags (the SDK's fx3_debug set; the SDK's headers are system headers)
add_library(fx3_flags INTERFACE)
target_compile_definitions(fx3_flags INTERFACE TX_ENABLE_EVENT_TRACE DEBUG CYU3P_FX3=1 __CYU3P_TX__=1)
target_include_directories(fx3_flags INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(fx3_flags SYSTEM INTERFACE "${SDK}/inc")
target_compile_options(fx3_flags INTERFACE -g ${FX3_OPT} -Wall -mcpu=arm926ej-s -marm -mthumb-interwork
                                           -ffunction-sections -fdata-sections -fno-strict-aliasing)

# Our sources' warnings: the host build's set (cmake/EinstarWarnings.cmake) as far as gcc has them
# (-Wmaybe-uninitialized for clang's -Wconditional-uninitialized). The SDK's own sources keep its flags.
# Every target builds with -Werror; `cmake --compile-no-warning-as-error` builds anyway.
set(FX3_WARNINGS -Wall -Wextra -Wpedantic
                 -Wconversion -Wsign-conversion
                 -Wuninitialized -Wmaybe-uninitialized
                 -Wmissing-field-initializers
                 -Wshadow
                 -Wswitch-enum -Wimplicit-fallthrough
                 -Wno-expansion-to-defined)
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

# the SDK's cyfxtx.c (heap / DMA buffers), used as-is
set(APP_SOURCES src/gpif_config.c src/globals.c src/sync.c src/version.c src/boot.c src/usb.c src/app.c
                src/sensor_i2c.c src/usb_descriptors.c "${SDK}/fw_build/cyfxtx.c" src/spi_flash.c
                src/gpio_led.c src/commands.c src/update.c src/common.c)
set(OUR_SOURCES ${APP_SOURCES})
list(FILTER OUR_SOURCES INCLUDE REGEX "^src/")
set_source_files_properties(${OUR_SOURCES} PROPERTIES COMPILE_OPTIONS "${FX3_WARNINGS}")

set(LIBS "${SDK}/lib/libcyu3sport.a" "${SDK}/lib/libcyu3lpp.a" "${SDK}/lib/libcyfxapi.a"
         "${SDK}/lib/libcyu3threadx.a" c gcc)

set(PACKAGES)
foreach(lang IN LISTS FW_LANGUAGES)
    set(out "${CMAKE_BINARY_DIR}/${lang}")
    set(elf "einstar_fx3_${lang}")
    add_executable(${elf} "${SDK}/fw_build/cyfx_gcc_startup.S" ${APP_SOURCES})
    target_link_libraries(${elf} PRIVATE fx3_flags ${LIBS})
    set_source_files_properties("${SDK}/fw_build/cyfx_gcc_startup.S" PROPERTIES COMPILE_OPTIONS "-mcpu=arm926ej-s;-marm")
    target_compile_definitions(${elf} PRIVATE FW_LANG="${lang}")
    set_target_properties(${elf} PROPERTIES OUTPUT_NAME einstar_fx3 SUFFIX ".elf" RUNTIME_OUTPUT_DIRECTORY "${out}"
                                            LINK_DEPENDS "${SDK}/fw_build/fx3_512k.ld")
    target_link_options(${elf} PRIVATE -mcpu=arm926ej-s -marm -nostartfiles -T "${SDK}/fw_build/fx3_512k.ld"
                        -Wl,--entry,CyU3PFirmwareEntry -Wl,--gc-sections -Wl,--no-wchar-size-warning
                        "-Wl,-Map,${out}/einstar_fx3.map")

    add_custom_command(OUTPUT "${out}/einstar_fx3.img"
        COMMAND "${HOST_BIN}/elf2img" -i "${out}/einstar_fx3.elf" -o "${out}/einstar_fx3.img" -i2cconf 1c -imgtype b0
        DEPENDS ${elf} "${HOST_BIN}/elf2img" VERBATIM)
    string(REPLACE "%LANG%" "${lang}" pkg_name "${PACKAGE_NAME}")
    add_custom_command(OUTPUT "${out}/${pkg_name}"
        COMMAND "${HOST_BIN}/mkpackage" "${out}/einstar_fx3.img" "${FPGA_BITSTREAM}" "${out}/${pkg_name}"
        DEPENDS "${out}/einstar_fx3.img" "${FPGA_BITSTREAM}" "${HOST_BIN}/mkpackage"
        COMMENT "Packaging ${pkg_name}" VERBATIM)
    list(APPEND PACKAGES "${out}/${pkg_name}")
endforeach()

add_custom_target(firmware ALL DEPENDS ${PACKAGES})
