/* globals.c -- application state shared between modules (the vendor's main-file .bss, in order). */
#include "app.h"

volatile AppState_t app_state = {0};
volatile uint32_t       app_active = 0;
CyU3PTimer     emc_wdg_timer = {0};
volatile uint8_t        emc_wdg_progress = 0;
volatile uint8_t        usb_ep_halted = 0;
volatile uint8_t        emc_wdg_limit = 0;
CyU3PSemaphore i2c_sem = {0};
CyU3PSemaphore flash_sem = {0};
uint8_t        hid_idle_rate_2 = 0;
uint8_t        hid_idle_rate_7 = 0;
volatile uint32_t       usb_unconfigured_secs = 0;
