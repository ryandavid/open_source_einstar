/* FPGA registers, reached over I2C (fpga_reg_read/write, 32-bit big-endian values). Roles as seen
 * from the commands that use them; "(?)" where only the number is known. */
#ifndef FPGA_REGS_H
#define FPGA_REGS_H

#define FPGA_REG_EXPOSURE        1
#define FPGA_REG_TRIGGER_PERIOD  2
#define FPGA_REG_TRIGGER_SWITCH  3
#define FPGA_REG_STROBE_0        4      /* strobe luminance, route 0 */
#define FPGA_REG_STROBE_1        5      /* strobe luminance, route 1 */
#define FPGA_REG_6               6      /* (?) */
#define FPGA_REG_7               7      /* (?) written after trigger-switch changes */
#define FPGA_REG_LD_BRIGHTNESS   8
#define FPGA_REG_STATE           9      /* buttons, device state (command 00/07) */
#define FPGA_REG_MODE            10     /* laser mode + status LED (fpga_set_mode) */
#define FPGA_REG_11              11     /* (?) */
#define FPGA_REG_CONTROL         12     /* (?) 1 by 00/EF, 4 by 10/7B */
#define FPGA_REG_13              13     /* (?) */
#define FPGA_REG_LD_WORKMODE     16
#define FPGA_REG_19              19     /* (?) cleared by 00/0D */

/* A register value is a big-endian word. Registers holding several fields (1: two exposures,
 * 13: three bytes) take a field byte with bit 7 set as "leave this field unchanged" (?). */
#define FPGA_FIELD_KEEP          ((uint8_t)0x80)

struct fpga_word {               /* a register value as sent: b0 = bits 31-24 .. b3 = bits 7-0 */
    uint8_t b0, b1, b2, b3;
};

#endif
