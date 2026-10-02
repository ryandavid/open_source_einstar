/* commands.c -- the command channel: reply header and the command dispatcher. */
#include "app.h"

/* Values the info commands report (fixed in the firmware) */
#define REPORTED_VID          0x3267    /* 10/00; also the USB VID */
#define REPORTED_PID          0x0001    /* 10/01; the USB PID is 0x0003 */
#define REPORTED_WIDTH        1280      /* 10/16 */
#define REPORTED_HEIGHT       1024      /* 10/17 */
#define REPORTED_EXPOSURE_MAX 10000     /* 10/20; 10/23 accepts up to EXPOSURE_LIMIT */
#define REPORTED_EXPOSURE_MIN 1         /* 10/21 */
#define REPORTED_GAIN_MAX     800       /* 10/24, percent */
#define REPORTED_GAIN_MIN     1         /* 10/25 */
#define REPORTED_PIXEL_BITS   8         /* 10/2E */
#define REPORTED_SENSORS      3         /* 10/51 */
#define REPORTED_0C           7         /* 00/0C (?) */
#define REPORTED_56           0x00100000 /* 10/56 (?) */
#define VENDOR_NAME_LEN       18
#define PRODUCT_NAME_LEN      12        /* the copy includes the terminating NUL: 13 bytes */
#define FW_VERSION_LEN        42
#define SERIAL_REPLY_LEN      12        /* 00/04 declares 12 bytes, fills 8 */
#define STATE_REPLY_LEN       20        /* 00/07 declares 20 bytes, fills 14 */

/* FX3 registers read directly. CC/CC returns the GPIO 50 and 52 pin registers (gpio_regs.h:
 * lpp_gpio_simple[n]; bit 1 is the input level); 00/04 the eFuse die ID (not in the SDK headers;
 * the address Cypress gives for the unique ID). */
#define FX3_GPIO_SIMPLE(n)    (*(volatile unsigned int *)(0xe0001100 + 4 * (n)))
#define FX3_DIE_ID_LO         (*(volatile unsigned int *)0xe0055010)
#define FX3_DIE_ID_HI         (*(volatile unsigned int *)0xe0055014)

/* FPGA_REG_EXPOSURE: two 15-bit exposures, cameras 1+2 (selector <= 3) in the low half, camera 4
 * in the high half; the other half is written as FPGA_FIELD_KEEP, 0. */
#define EXPOSURE_BITS         15
#define EXPOSURE_LOW_HALF_SEL 3         /* camera selectors 1, 2 (and 3) */
#define EXPOSURE_HI_MASK      0x7f      /* high byte of a 15-bit exposure */
#define EXPOSURE_LIMIT        50000     /* 10/23: larger values are ignored (no error) */

/* FPGA value registers (LD brightness, LD work mode, strobe): value << 2 | low 2 bits */
#define FPGA_VALUE_SHIFT      2
#define FPGA_LOW_BITS         3
#define LD_BRIGHTNESS_MAX     100       /* 10/68 clamps; the FPGA holds level / 2 */
#define LD_BRIGHTNESS_BIT31   0x80000000 /* (?) set by 10/68 */
#define LD_BRIGHTNESS_ENABLE  1         /* (?) low bits written by 10/68 */
#define STROBE_ENABLE         1         /* (?) low bits written by 10/70 */
#define STROBE0_LIMIT         1000000   /* 10/70 route limits: never reached by a 16-bit value */
#define STROBE1_LIMIT         5000000

/* FPGA_REG_7, written after every trigger-switch change (?) */
#define REG7_MASK             0x3fff
#define REG7_SHIFT            17
#define REG7_LOW              7

#define TRIGGER_PERIOD_MIN    1000      /* 10/49: values outside are ignored (no error) */
#define TRIGGER_PERIOD_MAX    1000000

/* FPGA_REG_6 (?): 10/3F writes REG6_BASE | a mode byte; 10/3E maps the low 2 bits back */
#define REG6_BASE             0x1f40
#define REG6_MODE1            0xf0
#define REG6_MODE2            0xf3

/* FPGA_REG_STATE (00/07 reply): six 2-bit button codes from bit 5, flags in bits 0-4, 17, 26 and
 * a byte from bit 18 */
#define STATE_BUTTON(n)       (5 + 2 * (n))
#define STATE_BUTTON_MASK     3
#define STATE_BIT17           17        /* also gates the EP 0x83 restart */
#define STATE_BYTE            18
#define STATE_BIT26           26

/* fpga_set_mode() laser mode for 10/62 DISTANCE 0, 1, 2 */
#define DISTANCE0_MODE        4
#define DISTANCE1_MODE        1
#define DISTANCE2_MODE        2

#define REG13_FIELD_A_SHIFT   8         /* 10/71 returns the byte 10/72 writes */
#define REG13_FIELD_MASK      0x7f

/* FPGA_REG_CONTROL values (?) */
#define CONTROL_EF            1
#define CONTROL_CLEAR_STATE   4         /* 10/7B, "ClearState" in docs/protocol-device.md */

#define ADT7420_REG_TEMP      0x00      /* temperature MSB, LSB */

uint8_t  vendor_name[VENDOR_NAME_LEN] = "Shining3D";     /* CMD_VENDOR_NAME */
uint32_t ld_brightness = 50;                /* FPGA reg 16 >> 2, set by CMD_LD_WORKMODE_WR */
uint16_t cmd_last_seq = SEQ_NONE;       /* sequence number of the last command (repeat detection) */

uint32_t ld_mode = 0;                       /* laser work mode, set by CMD_LD_WORKMODE_WR */
volatile uint32_t ep83_halt_cleared = 0;             /* set by CLEAR_FEATURE(EP 0x83), consumed by command 7 */
uint32_t cmd7_seen = 0;                     /* set by CMD_DEVICE_STATE, never read */

/* Reply status and data length (resp[PKT_STATUS..]): ST_OK and the reply length when the payload
 * length is as expected, else ST_BAD_LENGTH and 0. Returns the status. (The final else is
 * unreachable; it is the vendor's.) */
uint8_t reply_status(uint32_t length, uint32_t expected, uint32_t replyLength, uint8_t *resp)
{
    uint8_t result = 0;
    uint8_t *p = 0;

    p = resp + PKT_STATUS;

    if (length == expected) {
        *p++ = ST_OK;
        *p++ = (uint8_t)(replyLength >> 24);
        *p++ = (uint8_t)(replyLength >> 16);
        *p++ = (uint8_t)(replyLength >> 8);
        *p = (uint8_t)replyLength;
        result = ST_OK;
    } else if (length != expected) {
        *p++ = ST_BAD_LENGTH;
        *p++ = 0;
        *p++ = 0;
        *p++ = 0;
        *p = 0;
        result = ST_BAD_LENGTH;
    } else {
        *p++ = ST_5;
        *p++ = 0;
        *p++ = 0;
        *p++ = 0;
        *p = 0;
        result = ST_5;
    }
    return result;
}

/* The command dispatcher (packet layout: protocol.h). The reply header is written first; each
 * command then checks the payload length (reply_status()) and fills the reply data. Unknown keys
 * get ST_FAILED. A sequence number equal to the previous command's (other than 0xFE/0xFF) is a
 * retransmission: no reply, and the next command is accepted whatever its number. Returns the
 * number of reply bytes (9 + the reply length).
 *
 * Code-shape notes (they keep the build identical): the scratch variables are shared by all cases
 * and their declaration order fixes their stack slots (the three reassigned in branches --
 * ld_mode_sel, strobe_reg, exposure -- must come in that order); `(int)` in the command-7 bit
 * test, the trailing `default: break;` in the indication switch and the case/break shapes are
 * the vendor's. */
#define cmd cmdBuf.buffer
#define ARG(n)    cmd[PKT_PAYLOAD + (n)]     /* request payload byte n */
#define REPLY(n)  resp[PKT_DATA + (n)]       /* reply data byte n */

uint32_t command_dispatch(CyU3PDmaBuffer_t cmdBuf, uint8_t *resp)
{
    unsigned char seq;
    unsigned int strobe;
    unsigned char route;
    unsigned int status;
    unsigned char *dst;
    unsigned short key;
    unsigned int length;
    struct fpga_word w;          /* scratch big-endian word for fpga_reg_write() */
    unsigned int ld_word2;
    unsigned int sw_word2;
    unsigned int sw_mask2;
    unsigned int sw_word;
    unsigned int sw_mask;
    unsigned int reg6;
    unsigned int ld_word;
    unsigned char level;
    unsigned int ld_reg;
    float gain_f;
    unsigned short gain;
    unsigned char cam;
    unsigned int u32;            /* scratch */
    unsigned char ld_mode_sel;
    unsigned char strobe_reg;
    unsigned int exposure;       /* after ld_mode_sel and strobe_reg: see the note above */
    unsigned char temp_raw[4] = {0};   /* defined result if adt7420_read fails */

    w.b0 = 0;
    w.b1 = 0;
    w.b2 = 0;
    w.b3 = 0;
    dst = 0;
    status = 0;
    route = 0;
    strobe = 0;
    ld_mode_sel = 0;
    strobe_reg = 0;

    seq = cmd[PKT_SEQ];
    key = (unsigned short)(cmd[PKT_GROUP] << 8) + cmd[PKT_OPCODE];
    length = ((uint32_t)cmd[PKT_LENGTH] << 24) + ((uint32_t)cmd[PKT_LENGTH + 1] << 16) + ((uint32_t)cmd[PKT_LENGTH + 2] << 8) +
             cmd[PKT_LENGTH + 3];

    dst = resp;
    *dst++ = seq;
    *dst++ = PKT_REPLY_UNMASKED;
    *dst++ = cmd[PKT_GROUP];
    *dst = cmd[PKT_OPCODE];

    if (seq == cmd_last_seq && seq != SEQ_TRANSPORT_FF && seq != SEQ_TRANSPORT_FE) {
        cmd_last_seq = SEQ_NONE;
        return 0;
    }
    cmd_last_seq = seq;

    switch (key) {
    case CMD_CCCC:
        if (reply_status(length, 0, 8, resp) == 0) {
            dst = resp + PKT_DATA;
            *dst++ = (unsigned char)(FX3_GPIO_SIMPLE(GPIO_50) >> 24);
            *dst++ = (unsigned char)(FX3_GPIO_SIMPLE(GPIO_50) >> 16);
            *dst++ = (unsigned char)(FX3_GPIO_SIMPLE(GPIO_50) >> 8);
            *dst++ = (unsigned char)(FX3_GPIO_SIMPLE(GPIO_50));
            *dst++ = (unsigned char)(FX3_GPIO_SIMPLE(GPIO_IN_52) >> 24);
            *dst++ = (unsigned char)(FX3_GPIO_SIMPLE(GPIO_IN_52) >> 16);
            *dst++ = (unsigned char)(FX3_GPIO_SIMPLE(GPIO_IN_52) >> 8);
            *dst = (unsigned char)(FX3_GPIO_SIMPLE(GPIO_IN_52));
            break;
        }
        break;

    case CMD_ERASE_APP_HEADER:
        io_reconfigure();
        flash_erase_range(0, FLASH_SECTOR_SIZE);   /* rounded up to 64 KB block 0 */
        CyU3PThreadSleep(25);
        CyU3PDeviceReset(0);
        break;

    case CMD_EF:
        if (reply_status(length, 0, 0, resp) == 0) {
            w.b0 = 0;
            w.b1 = 0;
            w.b2 = 0;
            w.b3 = CONTROL_EF;
            fpga_reg_write(FPGA_REG_CONTROL, &w.b0);
            CyU3PThreadSleep(1000);
            break;
        }
        break;

    case CMD_0D:
        if (reply_status(length, 0, 0, resp) == 0) {
            w.b0 = 0;
            w.b1 = 0;
            w.b2 = 0;
            w.b3 = 0;
            fpga_reg_write(FPGA_REG_19, &w.b0);
            app_state.update_active = 0;
            app_state.reboot_after_reply = 0;
            break;
        }
        break;

    case CMD_BB:
        if (reply_status(length, 0, 4, resp) == 0) {
            u32 = fpga_reg_read(FPGA_REG_11);
            dst = resp + PKT_DATA;
            *dst++ = (unsigned char)(u32 >> 24);
            *dst++ = (unsigned char)(u32 >> 16);
            *dst++ = (unsigned char)(u32 >> 8);
            *dst = (unsigned char)u32;
            break;
        }
        break;

    case CMD_VENDOR_NAME:
        if (reply_status(length, 0, VENDOR_NAME_LEN, resp) == 0) {
            CyU3PMemCopy(resp + PKT_DATA, vendor_name, VENDOR_NAME_LEN);
            break;
        }
        break;

    case CMD_PRODUCT_NAME:
        if (reply_status(length, 0, PRODUCT_NAME_LEN, resp) == 0) {
            CyU3PMemCopy(resp + PKT_DATA, (uint8_t *)"EinScan10_01", PRODUCT_NAME_LEN + 1);
            break;
        }
        break;

    case CMD_02:
        if (reply_status(length, 0, 1, resp) == 0) {
            REPLY(0) = 0;
            break;
        }
        break;

    case CMD_03:
        if (reply_status(length, 0, 1, resp) == 0) {
            REPLY(0) = 1;
            break;
        }
        break;

    case CMD_FW_VERSION:
        if (reply_status(length, 0, FW_VERSION_LEN, resp) == 0) {
            CyU3PMemCopy(resp + PKT_DATA, fw_version, FW_VERSION_LEN);
            break;
        }
        break;

    case CMD_SERIAL:
        if (reply_status(length, 0, SERIAL_REPLY_LEN, resp) == 0) {
            dst = resp + PKT_DATA;
            *dst++ = (unsigned char)(FX3_DIE_ID_HI >> 24);
            *dst++ = (unsigned char)(FX3_DIE_ID_HI >> 16);
            *dst++ = (unsigned char)(FX3_DIE_ID_HI >> 8);
            *dst++ = (unsigned char)(FX3_DIE_ID_HI);
            *dst++ = (unsigned char)(FX3_DIE_ID_LO >> 24);
            *dst++ = (unsigned char)(FX3_DIE_ID_LO >> 16);
            *dst++ = (unsigned char)(FX3_DIE_ID_LO >> 8);
            *dst = (unsigned char)(FX3_DIE_ID_LO);
            break;
        }
        break;

    case CMD_VID:
        if (reply_status(length, 0, 2, resp) == 0) {
            REPLY(0) = (uint8_t)(REPORTED_VID >> 8);
            REPLY(1) = (uint8_t)REPORTED_VID;
            break;
        }
        break;

    case CMD_PID:
        if (reply_status(length, 0, 2, resp) == 0) {
            REPLY(0) = (uint8_t)(REPORTED_PID >> 8);
            REPLY(1) = (uint8_t)REPORTED_PID;
            break;
        }
        break;

    case CMD_MAX_WIDTH:
        if (reply_status(length, 1, 2, resp) == 0) {
            REPLY(0) = (uint8_t)(REPORTED_WIDTH >> 8);
            REPLY(1) = (uint8_t)REPORTED_WIDTH;
            break;
        }
        break;

    case CMD_MAX_HEIGHT:
        if (reply_status(length, 1, 2, resp) == 0) {
            REPLY(0) = (uint8_t)(REPORTED_HEIGHT >> 8);
            REPLY(1) = (uint8_t)REPORTED_HEIGHT;
            break;
        }
        break;

    case CMD_REBOOT:
        if (reply_status(length, 0, 0, resp) == 0) {
            app_state.reboot_after_reply = 1;
            break;
        }
        break;

    case CMD_VENDOR_NAME_LEN:
        if (reply_status(length, 0, 1, resp) == 0) {
            REPLY(0) = VENDOR_NAME_LEN;
            break;
        }
        break;

    case CMD_PRODUCT_NAME_LEN:
        if (reply_status(length, 0, 1, resp) == 0) {
            REPLY(0) = PRODUCT_NAME_LEN;
            break;
        }
        break;

    case CMD_FW_VERSION_LEN:
        if (reply_status(length, 0, 1, resp) == 0) {
            REPLY(0) = FW_VERSION_LEN;
            break;
        }
        break;

    case CMD_0C:
        if (reply_status(length, 0, 1, resp) == 0) {
            REPLY(0) = REPORTED_0C;
            break;
        }
        break;

    case CMD_MAX_EXPOSURE:
        if (reply_status(length, 1, 4, resp) == 0) {
            dst = resp + PKT_DATA;
            *dst++ = (uint8_t)(REPORTED_EXPOSURE_MAX >> 24);
            *dst++ = (uint8_t)(REPORTED_EXPOSURE_MAX >> 16);
            *dst++ = (uint8_t)(REPORTED_EXPOSURE_MAX >> 8);
            *dst++ = (uint8_t)REPORTED_EXPOSURE_MAX;
            break;
        }
        break;

    case CMD_MIN_EXPOSURE:
        if (reply_status(length, 1, 4, resp) == 0) {
            dst = resp + PKT_DATA;
            *dst++ = (uint8_t)(REPORTED_EXPOSURE_MIN >> 24);
            *dst++ = (uint8_t)(REPORTED_EXPOSURE_MIN >> 16);
            *dst++ = (uint8_t)(REPORTED_EXPOSURE_MIN >> 8);
            *dst++ = (uint8_t)REPORTED_EXPOSURE_MIN;
            break;
        }
        break;

    case CMD_MAX_GAIN:
        if (reply_status(length, 1, 2, resp) == 0) {
            REPLY(0) = (uint8_t)(REPORTED_GAIN_MAX >> 8);
            REPLY(1) = (uint8_t)REPORTED_GAIN_MAX;
            break;
        }
        break;

    case CMD_MIN_GAIN:
        if (reply_status(length, 1, 2, resp) == 0) {
            REPLY(0) = (uint8_t)(REPORTED_GAIN_MIN >> 8);
            REPLY(1) = (uint8_t)REPORTED_GAIN_MIN;
            break;
        }
        break;

    case CMD_GET_EXPOSURE:
        if (reply_status(length, 1, 4, resp) == 0) {
            status = 0;
            exposure = fpga_reg_read(FPGA_REG_EXPOSURE);
            if (ARG(0) <= EXPOSURE_LOW_HALF_SEL)
                exposure = (exposure << (32 - EXPOSURE_BITS)) >> (32 - EXPOSURE_BITS);
            else
                exposure = ((exposure >> 16) << (32 - EXPOSURE_BITS)) >> (32 - EXPOSURE_BITS);
            if (status == 0) {
                dst = resp + PKT_DATA;
                *dst++ = (unsigned char)(exposure >> 24);
                *dst++ = (unsigned char)(exposure >> 16);
                *dst++ = (unsigned char)(exposure >> 8);
                *dst = (unsigned char)exposure;
            } else {
                cmd_last_seq = SEQ_NONE;
                resp[PKT_STATUS] = ST_FAILED;
                break;
            }
        }
        break;

    case CMD_SET_EXPOSURE:
        if (reply_status(length, 5, 0, resp) == 0) {
            u32 = ((unsigned int)ARG(1) << 24) | ((unsigned int)ARG(2) << 16) |
                      ((unsigned int)ARG(3) << 8) | ARG(4);
            if (u32 <= EXPOSURE_LIMIT) {
                if (ARG(0) <= EXPOSURE_LOW_HALF_SEL) {
                    w.b0 = FPGA_FIELD_KEEP;
                    w.b1 = 0;
                    w.b2 = ARG(3) & EXPOSURE_HI_MASK;
                    w.b3 = ARG(4);
                } else {
                    w.b0 = ARG(3) & EXPOSURE_HI_MASK;
                    w.b1 = ARG(4);
                    w.b2 = FPGA_FIELD_KEEP;
                    w.b3 = 0;
                }
                fpga_reg_write(FPGA_REG_EXPOSURE, &w.b0);
                if (status != 0) {
                    cmd_last_seq = SEQ_NONE;
                    resp[PKT_STATUS] = ST_FAILED;
                }
            }
        }
        break;

    case CMD_GET_GAIN:
        if (reply_status(length, 1, 2, resp) == 0) {
            cam = ARG(0);
            gain = sensor_get_gain(cam);
            gain_f = gain;
            gain = (unsigned short)gain_f;
            REPLY(0) = (unsigned char)(unsigned short)(gain >> 8);
            REPLY(1) = (unsigned char)gain;
            cam = 0;
            break;
        }
        break;

    case CMD_SET_GAIN:
        if (reply_status(length, 3, 0, resp) == 0) {
            gain = ARG(1) << 8 | ARG(2);
            gain_f = gain;
            gain = (unsigned short)gain_f;
            cam = ARG(0);
            status = sensor_set_gain(cam, gain);
            if (status != 0) {
                cmd_last_seq = SEQ_NONE;
                resp[PKT_STATUS] = ST_FAILED;
            }
            cam = 0;
            break;
        }
        break;

    case CMD_PIXEL_BITS:
        if (reply_status(length, 1, 1, resp) == 0) {
            REPLY(0) = REPORTED_PIXEL_BITS;
            break;
        }
        break;

    case CMD_LD_BRIGHTNESS_RD:
        if (reply_status(length, 0, 1, resp) == 0) {
            ld_reg = fpga_reg_read(FPGA_REG_LD_BRIGHTNESS);
            level = (unsigned char)(ld_reg >> FPGA_VALUE_SHIFT);
            level = (unsigned char)(level << 1);
            REPLY(0) = level;
            break;
        }
        break;

    case CMD_LD_BRIGHTNESS_WR:
        if (reply_status(length, 1, 0, resp) == 0) {
            ld_word = 0;
            if (ARG(0) > LD_BRIGHTNESS_MAX)
                ARG(0) = LD_BRIGHTNESS_MAX;
            level = ARG(0);
            level = (unsigned char)(level >> 1);
            ld_word = level;
            ld_word = ld_word << FPGA_VALUE_SHIFT;
            ld_word = ld_word | LD_BRIGHTNESS_BIT31;
            ld_word = ld_word & ~(unsigned int)FPGA_LOW_BITS;
            ld_word = ld_word | LD_BRIGHTNESS_ENABLE;
            w.b0 = (unsigned char)(ld_word >> 24);
            w.b1 = (unsigned char)(ld_word >> 16);
            w.b2 = (unsigned char)(ld_word >> 8);
            w.b3 = (unsigned char)ld_word;
            fpga_reg_write(FPGA_REG_LD_BRIGHTNESS, &w.b0);
            break;
        }
        break;

    case CMD_STROBE_RD:
        if (reply_status(length, 1, 2, resp) == 0) {
            route = ARG(0);
            if (route == 0)
                strobe_reg = FPGA_REG_STROBE_0;
            else if (route == 1)
                strobe_reg = FPGA_REG_STROBE_1;
            strobe = fpga_reg_read(strobe_reg);
            strobe = strobe >> FPGA_VALUE_SHIFT;
            CyU3PDebugPrint(4, "Get Led Brightness 0x%x \r\n", strobe);
            REPLY(0) = (unsigned char)(strobe >> 8);
            REPLY(1) = (unsigned char)strobe;
            break;
        }
        break;

    case CMD_STROBE_WR:
        if (reply_status(length, 3, 0, resp) == 0) {
            route = ARG(0);
            strobe = ARG(1);
            strobe = strobe << 8;
            strobe = strobe + ARG(2);
            if (route == 0) {
                strobe_reg = FPGA_REG_STROBE_0;
                if (strobe > STROBE0_LIMIT) {
                    resp[PKT_STATUS] = ST_BAD_LENGTH;
                    break;
                }
            } else if (route == 1) {
                strobe_reg = FPGA_REG_STROBE_1;
                if (strobe > STROBE1_LIMIT) {
                    resp[PKT_STATUS] = ST_BAD_LENGTH;
                    break;
                }
            }
            CyU3PDebugPrint(4, "Set Led Brightness, write fpag data:0x%x\r\n", strobe);
            strobe = strobe << FPGA_VALUE_SHIFT;
            strobe = (unsigned short)strobe;
            strobe = strobe | STROBE_ENABLE;
            w.b0 = (unsigned char)(strobe >> 24);
            w.b1 = (unsigned char)(strobe >> 16);
            w.b2 = (unsigned char)(strobe >> 8);
            w.b3 = (unsigned char)strobe;
            fpga_reg_write(strobe_reg, &w.b0);
            break;
        }
        break;

    case CMD_36:
        reply_status(length, 0, 1, resp);
        break;

    case CMD_37:
        reply_status(length, 1, 0, resp);
        break;

    case CMD_GET_REG6_MODE:
        if (reply_status(length, 0, 1, resp) == 0) {
            reg6 = fpga_reg_read(FPGA_REG_6);
            switch (reg6 & 3) {
            case 0:
                REPLY(0) = 1;               /* REG6_MODE1 reads back as 0 */
                break;
            case 1:
                REPLY(0) = 2;               /* REG6_MODE2 reads back as 3: no reply byte */
                break;
            }
        }
        break;

    case CMD_SET_REG6_MODE:
        if (reply_status(length, 1, 0, resp) == 0) {
            w.b0 = 0;
            w.b1 = 0;
            w.b2 = (uint8_t)(REG6_BASE >> 8);
            w.b3 = (uint8_t)REG6_BASE;
            switch (ARG(1)) {
            case 1:
                w.b3 |= REG6_MODE1;
                break;
            case 2:
                w.b3 |= REG6_MODE2;
                break;
            }
            fpga_reg_write(FPGA_REG_6, &w.b0);
            break;
        }
        break;

    case CMD_TRIGGER_SWITCH_RD:
        if (reply_status(length, 0, 1, resp) == 0) {
            u32 = fpga_reg_read(FPGA_REG_TRIGGER_SWITCH);
            REPLY(0) = (unsigned char)u32;
            break;
        }
        break;

    case CMD_TRIGGER_SWITCH_WR:
        if (reply_status(length, 1, 0, resp) == 0) {
            sw_mask = 0;
            sw_word = 0;
            w.b0 = 0;
            w.b1 = 0;
            w.b2 = 0;
            w.b3 = ARG(0);
            fpga_reg_write(FPGA_REG_TRIGGER_SWITCH, &w.b0);
            sw_mask = REG7_MASK;
            sw_word = (sw_mask << REG7_SHIFT) | REG7_LOW;
            w.b0 = (unsigned char)(sw_word >> 24);
            w.b1 = (unsigned char)(sw_word >> 16);
            w.b2 = (unsigned char)(sw_word >> 8);
            w.b3 = (unsigned char)sw_word;
            fpga_reg_write(FPGA_REG_7, &w.b0);
            break;
        }
        break;

    case CMD_TRIGGER_WORD_RD:
        if (reply_status(length, 0, 4, resp) == 0) {
            u32 = fpga_reg_read(FPGA_REG_TRIGGER_SWITCH);
            dst = resp + PKT_DATA;
            *dst++ = (unsigned char)(u32 >> 24);
            *dst++ = (unsigned char)(u32 >> 16);
            *dst++ = (unsigned char)(u32 >> 8);
            *dst = (unsigned char)u32;
            break;
        }
        break;

    case CMD_TRIGGER_WORD_WR:
        if (reply_status(length, 4, 0, resp) == 0) {
            sw_mask2 = 0;
            sw_word2 = 0;
            w.b0 = ARG(0);
            w.b1 = ARG(1);
            w.b2 = ARG(2);
            w.b3 = ARG(3);
            fpga_reg_write(FPGA_REG_TRIGGER_SWITCH, &w.b0);
            sw_mask2 = REG7_MASK;
            sw_word2 = (sw_mask2 << REG7_SHIFT) | REG7_LOW;
            w.b0 = (unsigned char)(sw_word2 >> 24);
            w.b1 = (unsigned char)(sw_word2 >> 16);
            w.b2 = (unsigned char)(sw_word2 >> 8);
            w.b3 = (unsigned char)sw_word2;
            fpga_reg_write(FPGA_REG_7, &w.b0);
            break;
        }
        break;

    case CMD_LD_WORKMODE_WR:
        if (reply_status(length, 1, 0, resp) == 0) {
            ld_word2 = 0;
            if (ARG(0) == 0)
                ld_mode = 0;
            else
                ld_mode = 1;
            ld_brightness = fpga_reg_read(FPGA_REG_LD_WORKMODE) >> FPGA_VALUE_SHIFT;
            ld_word2 = (ld_brightness << FPGA_VALUE_SHIFT) | ld_mode;
            w.b0 = (unsigned char)(ld_word2 >> 24);
            w.b1 = (unsigned char)(ld_word2 >> 16);
            w.b2 = (unsigned char)(ld_word2 >> 8);
            w.b3 = (unsigned char)ld_word2;
            CyU3PDebugPrint(4, "Set LD Workmode rxbuf[8]=0x%x mode=%d brightness=%d\r\n", ARG(0), ld_mode, ld_brightness);
            fpga_reg_write(FPGA_REG_LD_WORKMODE, &w.b0);
            break;
        }
        break;

    case CMD_LD_WORKMODE_RD:
        if (reply_status(length, 0, 1, resp) == 0) {
            u32 = fpga_reg_read(FPGA_REG_LD_WORKMODE);
            u32 = u32 & FPGA_LOW_BITS;
            CyU3PDebugPrint(4, "LD work mode 0x%x \r\n", u32);
            REPLY(0) = (unsigned char)u32;
            break;
        }
        break;

    case CMD_TRIGGER_PERIOD_RD:
        if (reply_status(length, 0, 4, resp) == 0) {
            u32 = fpga_reg_read(FPGA_REG_TRIGGER_PERIOD);
            dst = resp + PKT_DATA;
            *dst++ = (unsigned char)(u32 >> 24);
            *dst++ = (unsigned char)(u32 >> 16);
            *dst++ = (unsigned char)(u32 >> 8);
            *dst = (unsigned char)u32;
            break;
        }
        break;

    case CMD_TRIGGER_PERIOD_WR:
        if (reply_status(length, 4, 0, resp) == 0) {
            w.b0 = ARG(0);
            w.b1 = ARG(1);
            w.b2 = ARG(2);
            w.b3 = ARG(3);
            u32 = ((unsigned int)w.b0 << 24) | ((unsigned int)w.b1 << 16) |
                      ((unsigned int)w.b2 << 8) | w.b3;
            if (u32 <= TRIGGER_PERIOD_MAX) {
                if (u32 >= TRIGGER_PERIOD_MIN) {
                    fpga_reg_write(FPGA_REG_TRIGGER_PERIOD, &w.b0);
                }
            }
        }
        break;

    case CMD_SENSOR_COUNT:
        if (reply_status(length, 0, 1, resp) == 0) {
            REPLY(0) = REPORTED_SENSORS;
            break;
        }
        break;

    case CMD_56:
        if (reply_status(length, 0, 4, resp) == 0) {
            dst = resp + PKT_DATA;
            *dst++ = (uint8_t)(REPORTED_56 >> 24);
            *dst++ = (uint8_t)(REPORTED_56 >> 16);
            *dst++ = (uint8_t)(REPORTED_56 >> 8);
            *dst = (uint8_t)REPORTED_56;
            break;
        }
        break;

    case CMD_COLOUR_MODE:
        if (reply_status(length, 1, 1, resp) == 0) {
            if (ARG(2) == 2) {
                REPLY(0) = 0;
            }
        }
        break;

    case CMD_DEVICE_STATE:
        if (reply_status(length, 0, STATE_REPLY_LEN, resp) == 0) {
            cmd7_seen = 1;
            u32 = fpga_reg_read(FPGA_REG_STATE);
            dst = resp + PKT_DATA;
            *dst++ = (unsigned char)((unsigned char)(u32 >> STATE_BUTTON(0)) & STATE_BUTTON_MASK);
            *dst++ = (unsigned char)((unsigned char)(u32 >> STATE_BUTTON(1)) & STATE_BUTTON_MASK);
            *dst++ = (unsigned char)((unsigned char)(u32 >> STATE_BUTTON(2)) & STATE_BUTTON_MASK);
            *dst++ = (unsigned char)((unsigned char)(u32 >> STATE_BUTTON(3)) & STATE_BUTTON_MASK);
            *dst++ = (unsigned char)((unsigned char)(u32 >> STATE_BUTTON(4)) & STATE_BUTTON_MASK);
            *dst++ = (unsigned char)((unsigned char)(u32 >> STATE_BUTTON(5)) & STATE_BUTTON_MASK);
            *dst++ = (unsigned char)((unsigned char)u32 & 1);
            *dst++ = (unsigned char)((unsigned char)(u32 >> 1) & 1);
            *dst++ = (unsigned char)((unsigned char)(u32 >> 2) & 1);
            *dst++ = (unsigned char)((unsigned char)(u32 >> 3) & 1);
            *dst++ = (unsigned char)((unsigned char)(u32 >> 4) & 1);
            *dst++ = (unsigned char)((unsigned char)(u32 >> STATE_BIT17) & 1);
            *dst++ = (unsigned char)(u32 >> STATE_BYTE);
            *dst = (unsigned char)((unsigned char)(u32 >> STATE_BIT26) & 1);
            if (ep83_halt_cleared == 1 && ((u32 >> STATE_BIT17) & 1) != 0) {
                ep83_halt_cleared = 0;
                app_restart();
                break;
            }
        }
        break;

    case CMD_INDICATION:
        if (reply_status(length, 2, 0, resp) == 0) {
            switch (ARG(0)) {
            case 0:
                ld_mode_sel = DISTANCE0_MODE;
                break;
            case 1:
                ld_mode_sel = DISTANCE1_MODE;
                break;
            case 2:
                ld_mode_sel = DISTANCE2_MODE;
                break;
            default:
                break;
            }
            fpga_set_mode(ld_mode_sel, FPGA_KEEP);
            break;
        }
        break;

    case CMD_REG13_RD:
        if (reply_status(length, 0, 1, resp) == 0) {
            u32 = fpga_reg_read(FPGA_REG_13);
            REPLY(0) = (unsigned char)((unsigned char)(u32 >> REG13_FIELD_A_SHIFT) & REG13_FIELD_MASK);
            break;
        }
        break;

    case CMD_TEMPERATURE:
        if (reply_status(length, 0, 2, resp) == 0) {
            status = adt7420_read(ADT7420_REG_TEMP, temp_raw);
            if (status != 0) {
                cmd_last_seq = SEQ_NONE;
                resp[PKT_STATUS] = ST_FAILED;
            }
            REPLY(0) = temp_raw[0];
            REPLY(1) = temp_raw[1];
            break;
        }
        break;

    case CMD_REG13_WR_A:
        if (reply_status(length, 1, 0, resp) == 0) {
            w.b0 = 0;
            w.b1 = FPGA_FIELD_KEEP;
            w.b2 = ARG(0);
            w.b3 = FPGA_FIELD_KEEP;
            u32 = ((unsigned int)w.b0 << 24) | ((unsigned int)w.b1 << 16) |
                      ((unsigned int)w.b2 << 8) | w.b3;
            status = fpga_reg_write(FPGA_REG_13, &w.b0);
            if (status != 0) {
                cmd_last_seq = SEQ_NONE;
                resp[PKT_STATUS] = ST_FAILED;
                break;
            }
        }
        break;

    case CMD_REG13_WR_B:
        if (reply_status(length, 1, 0, resp) == 0) {
            w.b0 = 0;
            w.b1 = FPGA_FIELD_KEEP;
            w.b2 = FPGA_FIELD_KEEP;
            w.b3 = ARG(0);
            u32 = ((unsigned int)w.b0 << 24) | ((unsigned int)w.b1 << 16) |
                      ((unsigned int)w.b2 << 8) | w.b3;
            status = fpga_reg_write(FPGA_REG_13, &w.b0);
            if (status != 0) {
                cmd_last_seq = SEQ_NONE;
                resp[PKT_STATUS] = ST_FAILED;
                break;
            }
        }
        break;

    case CMD_REG13_WR_C:
        if (reply_status(length, 1, 0, resp) == 0) {
            w.b0 = 0;
            w.b1 = ARG(0);
            w.b2 = FPGA_FIELD_KEEP;
            w.b3 = FPGA_FIELD_KEEP;
            u32 = ((unsigned int)w.b0 << 24) | ((unsigned int)w.b1 << 16) |
                      ((unsigned int)w.b2 << 8) | w.b3;
            status = fpga_reg_write(FPGA_REG_13, &w.b0);
            if (status != 0) {
                cmd_last_seq = SEQ_NONE;
                resp[PKT_STATUS] = ST_FAILED;
                break;
            }
        }
        break;

    case CMD_CLEAR_STATE:
        if (reply_status(length, 0, 0, resp) == 0) {
            w.b0 = 0;
            w.b1 = 0;
            w.b2 = 0;
            w.b3 = CONTROL_CLEAR_STATE;
            fpga_reg_write(FPGA_REG_CONTROL, &w.b0);
            CyU3PThreadSleep(10);
            break;
        }
        break;

    default:
        dst = resp + PKT_STATUS;
        *dst++ = ST_FAILED;
        *dst++ = 0;
        *dst++ = 0;
        *dst++ = 0;
        *dst = 0;
        break;

    case CMD_7A:
        break;

    case CMD_77:
        break;

    case CMD_78:
        break;
    }

    return (unsigned short)((unsigned short)((unsigned short)(resp[PKT_DATA_LENGTH + 2] << 8) |
                             resp[PKT_DATA_LENGTH + 3]) + PKT_DATA);
}
