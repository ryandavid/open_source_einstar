/* version.c -- the version string reported by command 5. */
#include "app.h"

#define FW_PRODUCT   "EinScan10_01"
#define FW_SENSOR    "SC130"
#define FW_VERSION   "2.10"
#define FPGA_VERSION "3.7"

/* The initial content is a stale default; build_version_string() overwrites it at boot. */
uint8_t fw_version[64] = "EinScan10_01_SC130_FX3_V0.7_FPGA_V2.6_CH";

/* "EinScan10_01_SC130_OPN_V2.10_FPGA_V3.7_<lang>": the vendor's "EinScan10_01_SC130_FX3_V2.10_..." with
 * the FX3 token replaced by OPN, marking this open build. Same length (42 bytes with the NUL, as 00/05
 * and 00/0B report), and the same fields where EXStar looks: it splits on '_' and compares tokens 4 and
 * 6 (V2.10, V3.7) with its own package, offering an update only when the package's are higher, so EXStar
 * neither offers to replace this build nor rejects it (docs/firmware.md 4.7). */
#define FW_BUILD_TAG "OPN"

void build_version_string(void)
{
    char buf[64] = {0};

    strcat(buf, FW_PRODUCT);
    strcat(buf, "_");
    strcat(buf, FW_SENSOR);
    strcat(buf, "_" FW_BUILD_TAG "_V");
    strcat(buf, FW_VERSION);
    strcat(buf, "_FPGA_V");
    strcat(buf, FPGA_VERSION);
    strcat(buf, "_");
    strcat(buf, FW_LANG);
    CyU3PMemCopy(fw_version, (uint8_t *)buf, strlen(buf));
}
