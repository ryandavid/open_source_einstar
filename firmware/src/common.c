/* common.c -- the vendor's uninitialised globals. They were tentative definitions (COMMON), which
 * ld allocates after all .bss input, i.e. after the libraries' .bss, in hash order of the vendor's
 * (unknown) names. Reproduced as one .bss section linked after the libraries (modules.txt), in the
 * vendor's address order. One section, so that the unreferenced 4 KB block survives --gc-sections. */
#include "app.h"

#define COMMON_BSS __attribute__((section(".bss.app_common")))

CyU3PThread     control_thread_obj   COMMON_BSS;
uint8_t         hid_set_report_buf[8]        COMMON_BSS;   /* EEPROM control-request data (usb_setup_cb copies 64 bytes!) */
uint8_t         hid_get_report_data[8]        COMMON_BSS;   /* length-prefixed reply block */
CyU3PDmaChannel ch_bulk_in           COMMON_BSS;
CyU3PDmaChannel ch_bulk_out          COMMON_BSS;
CyU3PTimer      update_timer         COMMON_BSS;
CyU3PThread     fpga_boot_thread_obj COMMON_BSS;
CyU3PThread     com_thread_obj       COMMON_BSS;
uint8_t         unreferenced_4k[4096] COMMON_BSS;  /* referenced by nothing in the linked code */
CyU3PDmaChannel ch_ctrl_out          COMMON_BSS;
CyU3PDmaChannel ch_image             COMMON_BSS;
CyU3PDmaChannel ch_ctrl_in           COMMON_BSS;
CyU3PThread     emc_wdg_thread_obj   COMMON_BSS;
