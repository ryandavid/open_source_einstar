"""Where the emulator finds its inputs. None of them are in the repository: the FX3 SDK, the firmware
builds and EXStar's update packages live in the untracked `.re/` working area (or wherever the
environment variables point)."""
import os

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
RE = os.environ.get("EINSTAR_RE", os.path.join(REPO, ".re"))

# The firmware tree (now tracked at the repo root; its sdk/, fpga/, reference/ are user-supplied and
# gitignored -- present on disk but not committed).
FIRMWARE = os.environ.get("FX3_FIRMWARE", os.path.join(REPO, "firmware"))
SDK_INC = os.path.join(FIRMWARE, "sdk", "inc")
SDK_LIB = os.path.join(FIRMWARE, "sdk", "lib")
NEWLIB_INC = os.path.join(FIRMWARE, "sdk", "newlib", "include")

# Any arm-none-eabi gcc, only used to compute SDK struct field offsets from the headers
FX3_GCC = os.environ.get("FX3_GCC",
    "/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi/bin/arm-none-eabi-gcc")

EXSTAR_FW_DIR = os.environ.get("EXSTAR_FW_DIR", "/Applications/EXStar.app/Contents/MacOS/fabu_UPDATE/Configure")
VENDOR_PACKAGE = os.path.join(EXSTAR_FW_DIR, "EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN_IAP.img")

# ELF identical to the vendor image, used only to name the vendor image's addresses (firmware/reference)
VENDOR_NAMES_ELF = os.environ.get("FX3_VENDOR_NAMES", os.path.join(FIRMWARE, "reference", "vendor-names.elf"))

# the candidate build to test: the top-level build's firmware output (EINSTAR_BUILD_FIRMWARE), in build/
# unless FX3_MODERN_ELF points elsewhere (firmware/check.sh sets it from its build directory)
MODERN_ELF = os.environ.get("FX3_MODERN_ELF", os.path.join(REPO, "build", "firmware", "EN", "einstar_fx3.elf"))

CACHE = os.environ.get("FX3EMU_CACHE", os.path.join(RE, "cache", "fx3emu"))


def build_elf(build_dir=None, lang="EN"):
    if build_dir:
        return os.path.join(build_dir, lang, "einstar_fx3.elf")
    return MODERN_ELF
