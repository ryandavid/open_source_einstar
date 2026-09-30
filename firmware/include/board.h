/* Board-level constants shared between modules. */
#ifndef BOARD_H
#define BOARD_H

/* GPIOs */
#define GPIO_I2C_SEL0        23        /* 4-bit I2C target select, see i2c_select_target() */
#define GPIO_I2C_SEL1        25
#define GPIO_I2C_SEL2        26
#define GPIO_I2C_SEL3        27
#define GPIO_FPGA_IRQ        36        /* input, rising-edge interrupt (gpio_interrupt_cb) */
#define GPIO_FPGA_PROGRAM    37        /* pulsed low to start FPGA configuration */
#define GPIO_HEARTBEAT       38
#define GPIO_FPGA_CFG_SEL    51        /* low: the FPGA takes the SPI data as its bitstream */
#define GPIO_50              50        /* not configured; only read back raw by command CC/CC */
#define GPIO_IN_52           52        /* input; only read back raw by command CC/CC */

/* IO matrix gpioSimpleEn[1] (GPIOs 32..63). The I2C select and heartbeat pins are unused GPIF
 * pins, taken with CyU3PDeviceGpioOverride() instead. */
#define GPIO_SIMPLE_EN_HI   ((1u << (GPIO_FPGA_IRQ - 32)) | (1u << (GPIO_FPGA_PROGRAM - 32)) | \
                             (1u << (GPIO_FPGA_CFG_SEL - 32)) | (1u << (GPIO_IN_52 - 32)))

/* USB endpoints */
#define EP_CTRL_OUT    0x01            /* commands in (control_thread) */
#define EP_CTRL_IN     0x81            /* command replies */
#define EP_BULK_OUT    0x02            /* bulk channel in (com_thread) */
#define EP_BULK_IN     0x82
#define EP_IMAGE_IN    0x83            /* GPIF II -> USB image stream */

/* I2C bus targets for i2c_select_target() (gpio_led.c drives the select GPIOs). The camera numbers
 * follow the vendor's GPIO_LVDS_CAM1/2/4 names. */
#define I2C_TARGET_NONE     0
#define I2C_TARGET_CAM1     1
#define I2C_TARGET_CAM2     2
#define I2C_TARGET_CAM4     4
#define I2C_TARGET_FPGA     5
#define I2C_TARGET_LIGHT    7
#define I2C_TARGET_ADT7420  8

/* fpga_set_mode(): leave that field unchanged */
#define FPGA_KEEP           0xFF
#define FPGA_LED_OK         2
#define FPGA_LED_FAULT      4

/* SPI flash (at least 5 MB used). Layout: docs/firmware.md section 3. */
#define FLASH_PAGE_SIZE     0x100        /* program page */
#define FLASH_SECTOR_SIZE   0x1000       /* erase sector */
#define FLASH_SECTOR_SHIFT  12
#define SLOT_A_APP          0x040000     /* A/B application images */
#define SLOT_B_APP          0x1C0000
#define SLOT_A_FPGA         0x080000     /* A/B FPGA bitstreams, paired with the images */
#define SLOT_B_FPGA         0x200000
#define BOOT_RECORD_PAGE    1008         /* 0x3F000: boot record (selects the slot) */
#define BOOT_RECORD_SECTOR  0x3F
#define USER_AREA_SECTOR    1024         /* 0x400000: 256 user pages (10/57, 10/58) */
#define USER_AREA_PAGES     256

#endif
