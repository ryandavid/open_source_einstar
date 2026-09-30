/* version.c -- the version string reported by command 5. */
#include "app.h"

#define FW_PRODUCT   "EinScan10_01"
#define FW_SENSOR    "SC130"
#define FW_VERSION   "2.10"
#define FPGA_VERSION "3.7"

/* The initial content is a stale default; build_version_string() overwrites it at boot. */
uint8_t fw_version[64] = "EinScan10_01_SC130_FX3_V0.7_FPGA_V2.6_CH";

/* "EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_<lang>" */
void build_version_string(void)
{
    char buf[64] = {0};

    strcat(buf, FW_PRODUCT);
    strcat(buf, "_");
    strcat(buf, FW_SENSOR);
    strcat(buf, "_FX3_V");
    strcat(buf, FW_VERSION);
    strcat(buf, "_FPGA_V");
    strcat(buf, FPGA_VERSION);
    strcat(buf, "_");
    strcat(buf, FW_LANG);
    CyU3PMemCopy(fw_version, (uint8_t *)buf, strlen(buf));
}
