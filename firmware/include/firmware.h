/* Einstar FX3 application: functions and data shared between modules, by module. */
#ifndef FIRMWARE_H
#define FIRMWARE_H

/* ---- globals.c: application state ------------------------------------------------------------ */
typedef struct {
    uint8_t  reboot_after_reply;   /* CMD_REBOOT: control thread resets the device after replying */
    uint8_t  usb_powered;          /* VBUS valid / removed events */
    uint8_t  unused2;
    uint8_t  usb_configured;       /* SET_CONFIGURATION seen; cleared on reset/suspend */
    uint32_t update_size;          /* firmware update: announced size */
    uint32_t update_active;        /* firmware update in progress (pauses the EMC watchdog) */
    uint8_t  update_done;          /* update written: the EMC watchdog thread reboots */
} AppState_t;

extern volatile AppState_t app_state;
extern volatile uint32_t       app_active;             /* endpoints and DMA channels are up */
extern CyU3PTimer     emc_wdg_timer;          /* 1 s, USB watchdog progress */
extern volatile uint8_t        emc_wdg_progress;       /* stays 0: emc_wdg_timer is never started */
extern volatile uint8_t        usb_ep_halted;          /* SuperSpeed endpoint reset seen */
extern volatile uint8_t        emc_wdg_limit;          /* never written: 0 */
extern CyU3PSemaphore i2c_sem;
extern CyU3PSemaphore flash_sem;              /* never created, see CyFxApplicationDefine() */
extern uint8_t        hid_idle_rate_2, hid_idle_rate_7;
extern volatile uint32_t       usb_unconfigured_secs;

/* ---- common.c: the vendor's COMMON variables ----------------------------------------------- */
extern CyU3PThread     control_thread_obj, fpga_boot_thread_obj, com_thread_obj, emc_wdg_thread_obj;
extern CyU3PDmaChannel ch_ctrl_in;            /* EP 0x81, command replies */
extern CyU3PDmaChannel ch_ctrl_out;           /* EP 0x01, commands */
extern CyU3PDmaChannel ch_bulk_in;            /* EP 0x82 */
extern CyU3PDmaChannel ch_bulk_out;           /* EP 0x02 */
extern CyU3PDmaChannel ch_image;              /* EP 0x83, GPIF II -> USB */
extern CyU3PTimer      update_timer;          /* firmware update: restarted by every page */
#define UPDATE_TIMEOUT_MS      2000
extern uint8_t         hid_set_report_buf[8], hid_get_report_data[8];

/* ---- gpif_config.c, usb_descriptors.c --------------------------------------------------------- */
extern const CyU3PGpifConfig_t CyFxGpifConfig;
extern const uint8_t CyFxUSB20DeviceDscr[], CyFxUSB30DeviceDscr[], CyFxUSBDeviceQualDscr[];
extern const uint8_t CyFxUSBFSConfigDscr[], CyFxUSBHSConfigDscr[], CyFxUSBBOSDscr[];
extern const uint8_t CyFxUSBSSConfigDscr[], CyFxUSBStringLangIDDscr[];
extern const uint8_t CyFxUSBManufactureDscr[], CyFxUSBProductDscr[];

/* ---- sync.c ----------------------------------------------------------------------------------- */
void i2c_lock(void);
void i2c_unlock(void);
void flash_lock(void);
void flash_unlock(void);
void emc_wdg_timer_cb(uint32_t arg);
void update_timer_cb(void);

/* ---- version.c -------------------------------------------------------------------------------- */
extern uint8_t fw_version[64];
void build_version_string(void);

/* ---- boot.c ----------------------------------------------------------------------------------- */
void eeprom_invalidate_cy(void);
void debug_console_init(void);
void gpif_start(void);

/* ---- usb.c ------------------------------------------------------------------------------------ */
void app_start(void);
void app_stop(void);
CyBool_t usb_lpm_request_cb(CyU3PUsbLinkPowerMode link_mode);
void stream_reset(void);
void app_restart(void);
void usb_event_cb(CyU3PUsbEventType_t evtype, uint16_t evdata);
void usb_ep_event_cb(CyU3PUsbEpEvtType evType, CyU3PUSBSpeed_t usbSpeed, uint8_t epNum);
CyBool_t usb_setup_cb(uint32_t setupdat0, uint32_t setupdat1);
void usb_init(void);
void i2c_init(void);
CyU3PReturnStatus_t io_matrix_init(void);
CyU3PReturnStatus_t io_reconfigure(void);

/* ---- app.c ------------------------------------------------------------------------------------ */
void fpga_boot_thread(uint32_t input);
void com_thread(uint32_t input);
void emc_wdg_thread(uint32_t input);
void control_thread(uint32_t input);
void CyFxApplicationDefine(void);
int main(void);

/* ---- sensor_i2c.c ----------------------------------------------------------------------------- */
CyU3PReturnStatus_t adt7420_read(uint8_t reg, uint8_t *data);
CyU3PReturnStatus_t fpga_reg_write(uint8_t reg, uint8_t *data);
uint32_t fpga_reg_read(uint8_t reg);
CyU3PReturnStatus_t sensor_reg_write(uint16_t reg, uint8_t val);
CyU3PReturnStatus_t sc130_reg_write(uint16_t reg, uint8_t val);
CyU3PReturnStatus_t light_reg_write(uint8_t reg, uint8_t value, uint8_t target);
CyU3PReturnStatus_t light_reg_write_raw(uint8_t reg, uint8_t value, uint8_t target);
void light_init(void);
void sc132_set_gain_linear_mode(void);
CyU3PReturnStatus_t sensor_set_gain(uint8_t cam, uint16_t percent);
uint16_t sensor_get_gain(uint8_t cam);
CyU3PReturnStatus_t sensors_init(void);
int fpga_set_mode(uint8_t ld_mode, uint8_t led);

/* ---- spi_flash.c ------------------------------------------------------------------------------ */
CyU3PReturnStatus_t flash_select(uint8_t dev, CyBool_t high);
CyU3PReturnStatus_t spi_init(void);
CyU3PReturnStatus_t flash_wait_ready(uint8_t dev);
CyU3PReturnStatus_t flash_erase_sector(uint16_t sector, uint8_t dev);
CyU3PReturnStatus_t flash_erase_block(uint8_t block, uint8_t dev);
int flash_erase_range(uint8_t dev, uint32_t size);
CyU3PReturnStatus_t flash_rw_pages(uint16_t page, uint16_t size, uint8_t *buffer, uint8_t dev, CyBool_t isRead);
CyU3PReturnStatus_t fpga_load(void);
int flash_write_status_reg(uint8_t cmd, uint8_t value, uint8_t isVolatile);
int flash_read_status_regs(uint8_t *sr);
void gpio_interrupt_cb(uint8_t gpioId);

/* ---- gpio_led.c ------------------------------------------------------------------------------- */
CyU3PReturnStatus_t gpio_init(void);
CyU3PReturnStatus_t i2c_select_target(uint8_t target);
void heartbeat_toggle(void);

/* ---- commands.c ------------------------------------------------------------------------------- */
extern volatile uint32_t ep83_halt_cleared;
uint8_t reply_status(uint32_t length, uint32_t expected, uint32_t replyLength, uint8_t *resp);
uint32_t command_dispatch(CyU3PDmaBuffer_t cmd, uint8_t *resp);

/* ---- update.c --------------------------------------------------------------------------------- */
int bulk_dispatch(void *req, void *resp);
uint32_t fpga_load_addr_get(void);

/* ---- SDK ---------------------------------------------------------------------------------------- */
/* In the library but not declared in the SDK 1.3.4 headers. The vendor called it undeclared
 * (implicitly returning int), which is what keeps their code; declared the same way. */
extern int CyU3PDmaChannelSendData(CyU3PDmaChannel *handle, uint8_t *buffer, uint16_t count);

#endif
