/* sensor_i2c.c -- everything on the I2C bus: image sensors (SC130GS/SC132GS, one per camera,
 * selected with i2c_select_target()), the FPGA's register bridge, the ADT7420 temperature sensor
 * and the light (laser/LED) driver. */
#include "app.h"

/* 8-bit I2C addresses */
#define I2C_ADT7420_WR   0x90
#define I2C_ADT7420_RD   0x91
#define I2C_FPGA_WR      0x20
#define I2C_FPGA_RD      0x21
#define I2C_SENSOR_WR    0x64
#define I2C_SENSOR_RD    0x65
#define I2C_LIGHT_WR     0xA6

/* Image-sensor registers */
#define SC_REG_EXP_HI    0x3e01
#define SC_REG_EXP_LO    0x3e02
#define SC_REG_GAIN_MODE 0x3e03
#define SC_GAIN_MODE_LINEAR 3        /* SC132 */
#define SC_REG_GAIN_HI   0x3e08
#define SC_REG_GAIN_LO   0x3e09
#define SC_TABLE_END     0x0001
#define SC_GAIN_UNITY    32.0f       /* gain register value for 1x (100 %) */

#define I2C_RETRIES      3           /* CyU3PI2cTransmitBytes retryCount, where used */

/* Light driver (I2C 0xA6, chip unidentified): light_init() writes 0x32 to register 0x10, 0xFF to
 * 0xB0, and 3 to 0xA0 through light_reg_write(), which sends 0xC0 | value << 3. */
#define LIGHT_DATA_BASE  0xC0
#define LIGHT_DATA_SHIFT 3

/* FPGA_REG_MODE (the last byte of the word): laser mode in bits 4-6, status LED in bits 0-2,
 * "leave unchanged" flags in bits 7 and 3. */
#define FPGA_MODE_FIELD      7
#define FPGA_MODE_LD_SHIFT   4
#define FPGA_MODE_LD_KEEP    0x80        /* bit 7 (the vendor wrote 0xFFFFFF80; the byte keeps 0x80) */
#define FPGA_MODE_LED_KEEP   8


typedef struct {
    uint16_t reg;
    uint8_t  val;
} sensor_reg_t;    /* 4 bytes, as the vendor's table: val is followed by a padding byte */

/* SC130GS initialisation: {register, value}, written to each camera by sensors_init(); ends at register 0x0001 */
const sensor_reg_t sc130_init_regs[111] = {
    {0x0103, 0x01}, {0x0100, 0x00}, {0x3000, 0x00}, {0x3001, 0x00},
    {0x3018, 0x70}, {0x3019, 0x00}, {0x3022, 0x19}, {0x302b, 0x80},
    {0x3030, 0x04}, {0x3031, 0x08}, {0x3034, 0x25}, {0x3035, 0x42},
    {0x3038, 0x44}, {0x3039, 0x10}, {0x303a, 0x2a}, {0x303b, 0x01},
    {0x303c, 0x04}, {0x303f, 0x01}, {0x3202, 0x00}, {0x3203, 0x00},
    {0x3205, 0x8b}, {0x3206, 0x02}, {0x3207, 0x04}, {0x320a, 0x04},
    {0x320b, 0x00}, {0x320c, 0x06}, {0x320d, 0x3e}, {0x320e, 0x02},
    {0x320f, 0x0e}, {0x3211, 0x0c}, {0x3213, 0x04}, {0x3221, 0x00},
    {0x3225, 0x02}, {0x3300, 0x20}, {0x3302, 0x0c}, {0x3306, 0x48},
    {0x3308, 0x50}, {0x330a, 0x01}, {0x330b, 0x20}, {0x330e, 0x1a},
    {0x3310, 0xf0}, {0x3311, 0x10}, {0x3319, 0xe8}, {0x3333, 0x90},
    {0x3334, 0x30}, {0x3348, 0x02}, {0x3349, 0xee}, {0x334a, 0x02},
    {0x334b, 0xe8}, {0x335d, 0x00}, {0x3380, 0xff}, {0x3382, 0xe0},
    {0x3383, 0x0a}, {0x3384, 0xe4}, {0x3400, 0x53}, {0x3416, 0x31},
    {0x3518, 0x07}, {0x3519, 0xc8}, {0x3620, 0x23}, {0x3621, 0x08},
    {0x3622, 0x06}, {0x3623, 0x14}, {0x3624, 0x20}, {0x3625, 0x00},
    {0x3627, 0x01}, {0x3630, 0x73}, {0x3632, 0x74}, {0x3633, 0x62},
    {0x3634, 0xff}, {0x3635, 0x44}, {0x3638, 0x82}, {0x3639, 0x74},
    {0x363a, 0x64}, {0x363b, 0x00}, {0x3640, 0x03}, {0x3654, 0x48},
    {0x3658, 0x9a}, {0x3663, 0x88}, {0x3664, 0x07}, {0x3c00, 0x41},
    {0x3d08, 0x00}, {0x3e01, 0x00}, {0x3e02, 0x00}, {0x3e03, 0x0b},
    {0x3e08, 0x00}, {0x3e09, 0xa0}, {0x3e0e, 0x00}, {0x3e0f, 0x14},
    {0x3e14, 0xb0}, {0x3f08, 0x04}, {0x4501, 0xc0}, {0x4502, 0x16},
    {0x4b00, 0xb2}, {0x4b01, 0x12}, {0x4b1b, 0x7f}, {0x5000, 0x01},
    {0x5b00, 0x02}, {0x5b01, 0x03}, {0x5b02, 0x01}, {0x5b03, 0x01},
    {0x3234, 0xa3}, {0x3228, 0x60}, {0x0100, 0x01}, {0x0100, 0x01},
    {0x0100, 0x01}, {0x0100, 0x01}, {0x0100, 0x01}, {0x0100, 0x01},
    {0x0100, 0x01}, {0x0100, 0x01}, {0x0001, 0x00}
};




/* ADT7420 temperature: `reg` selects the register, two bytes are read. */
CyU3PReturnStatus_t adt7420_read(uint8_t reg, uint8_t *data)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    preamble.length = 3;
    preamble.buffer[0] = I2C_ADT7420_WR;
    preamble.buffer[1] = reg;
    preamble.buffer[2] = I2C_ADT7420_RD;
    preamble.ctrlMask = 0x0002;             /* repeated start before byte 2 */

    if (data == NULL)
        return CY_U3P_ERROR_BAD_POINTER;

    i2c_lock();
    i2c_select_target(I2C_TARGET_ADT7420);
    status = CyU3PI2cReceiveBytes(&preamble, data, 2, 0);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "Failed to ReadI2C_ADT7420\r\n", status);
    }
    i2c_select_target(I2C_TARGET_NONE);
    i2c_unlock();
    return status;
}

/* FPGA register write: 4 data bytes (big-endian value). */
CyU3PReturnStatus_t fpga_reg_write(uint8_t reg, uint8_t *data)
{
    CyU3PReturnStatus_t status;
    CyU3PI2cPreamble_t preamble;

    status = CY_U3P_SUCCESS;
    preamble.length = 2;
    preamble.buffer[0] = I2C_FPGA_WR;
    preamble.buffer[1] = reg;
    preamble.ctrlMask = 0;

    i2c_lock();
    i2c_select_target(I2C_TARGET_FPGA);

    status = CyU3PI2cTransmitBytes(&preamble, data, 4, I2C_RETRIES);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "write data To FPGA Failed %d\r\n", status);
    }

    i2c_select_target(I2C_TARGET_NONE);
    i2c_unlock();
    return status;
}

/* FPGA register read: 4 bytes, big-endian; retried once, the status is not returned. */
uint32_t fpga_reg_read(uint8_t reg)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t value;
    uint8_t data[4] = {0};       /* defined result if the read fails */
    CyU3PI2cPreamble_t preamble;

    preamble.length = 3;
    preamble.buffer[0] = I2C_FPGA_WR;
    preamble.buffer[1] = reg;
    preamble.buffer[2] = I2C_FPGA_RD;

    i2c_lock();
    i2c_select_target(I2C_TARGET_FPGA);
    preamble.ctrlMask = 0x0002;
    status = CyU3PI2cReceiveBytes(&preamble, data, 4, 0);
    if (status != CY_U3P_SUCCESS) {
        status = CyU3PI2cReceiveBytes(&preamble, data, 4, 0);
    }
    value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    i2c_select_target(I2C_TARGET_NONE);
    i2c_unlock();
    return value;
}

/* Image-sensor register write on the currently selected camera (caller holds the target). */
CyU3PReturnStatus_t sensor_reg_write(uint16_t reg, uint8_t val)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t data = val;
    CyU3PI2cPreamble_t preamble;

    preamble.length = 3;
    preamble.buffer[0] = I2C_SENSOR_WR;
    preamble.buffer[1] = (uint8_t)((uint16_t)(reg >> 8) & 0xff);
    preamble.buffer[2] = (uint8_t)(reg & 0xff);
    preamble.ctrlMask = 0;

    i2c_lock();
    status = CyU3PI2cTransmitBytes(&preamble, &data, 1, 0);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "WriteI2C_seneor_register err, write data\357\274\2320x%x 0x%x\r\n", reg, data);
    }
    i2c_unlock();
    return status;
}

/* Same as sensor_reg_write(), logging every write. */
CyU3PReturnStatus_t sc130_reg_write(uint16_t reg, uint8_t val)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    preamble.length = 3;
    preamble.buffer[0] = I2C_SENSOR_WR;
    preamble.buffer[1] = (uint8_t)((uint16_t)(reg >> 8) & 0xff);
    preamble.buffer[2] = (uint8_t)(reg & 0xff);
    preamble.ctrlMask = 0;

    CyU3PDebugPrint(4, "Reg write: 0x%x\r\n", val);
    i2c_lock();
    status = CyU3PI2cTransmitBytes(&preamble, &val, 1, 0);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "WriteI2C_seneor_register_for_sc130 err, write data:0x%x 0x%x\r\n", reg, val);
    }
    i2c_unlock();
    return status;
}

/* Light driver register write; the data byte is 0xC0 | (value << 3). */
CyU3PReturnStatus_t light_reg_write(uint8_t reg, uint8_t value, uint8_t target)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t data[1];
    CyU3PI2cPreamble_t preamble;

    i2c_lock();
    i2c_select_target(target);

    preamble.length = 2;
    preamble.buffer[0] = I2C_LIGHT_WR;
    preamble.buffer[1] = reg;
    preamble.ctrlMask = 0;

    data[0] = (value << LIGHT_DATA_SHIFT) | LIGHT_DATA_BASE;

    status = CyU3PI2cTransmitBytes(&preamble, data, 1, I2C_RETRIES);

    i2c_select_target(I2C_TARGET_NONE);
    i2c_unlock();

    CyU3PDebugPrint(4, "WriteI2C_Light Data 0x%x to Addr 0x%x\r\n", data[0], reg);

    return status;
}

/* Light driver register write, raw data byte, retried once. */
CyU3PReturnStatus_t light_reg_write_raw(uint8_t reg, uint8_t value, uint8_t target)
{
    CyU3PReturnStatus_t status;
    uint8_t data[1];
    CyU3PI2cPreamble_t preamble;

    status = CY_U3P_SUCCESS;
    i2c_lock();
    i2c_select_target(target);

    preamble.length = 2;
    preamble.buffer[0] = I2C_LIGHT_WR;
    preamble.buffer[1] = reg;
    preamble.ctrlMask = 0;
    data[0] = value;

    status = CyU3PI2cTransmitBytes(&preamble, data, 1, 0);
    if (status != CY_U3P_SUCCESS) {
        status = CyU3PI2cTransmitBytes(&preamble, data, 1, 0);
    }

    i2c_select_target(I2C_TARGET_NONE);
    i2c_unlock();
    return status;
}

void light_init(void)
{
    light_reg_write_raw(0x10, 0x32, I2C_TARGET_LIGHT);
    light_reg_write_raw(0xB0, 0xFF, I2C_TARGET_LIGHT);
    light_reg_write(0xA0, 3, I2C_TARGET_LIGHT);
}

/* Linear gain mode (SC132) on all three cameras. */
void sc132_set_gain_linear_mode(void)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    i2c_select_target(I2C_TARGET_CAM1);
    status = sensor_reg_write(SC_REG_GAIN_MODE, SC_GAIN_MODE_LINEAR);
    CyU3PDebugPrint(4, "SetSc132GainLinearMode GPIO_LVDS_CAM1:%d\n\r", status);
    i2c_select_target(I2C_TARGET_CAM2);
    status = sensor_reg_write(SC_REG_GAIN_MODE, SC_GAIN_MODE_LINEAR);
    CyU3PDebugPrint(4, "SetSc132GainLinearMode GPIO_LVDS_CAM2:%d\n\r", status);
    i2c_select_target(I2C_TARGET_CAM4);
    status = sensor_reg_write(SC_REG_GAIN_MODE, SC_GAIN_MODE_LINEAR);
    CyU3PDebugPrint(4, "SetSc132GainLinearMode GPIO_LVDS_CAM4:%d\n\r", status);
    i2c_select_target(I2C_TARGET_NONE);
}

#define HIBYTE(x) ((uint8_t)(uint16_t)((x) >> 8))

/* Gain in percent -> register value in 1/32 steps (nearest, exact halves down), written to 0x3e08/0x3e09.
 * Camera 4 uses the logging writer. Bug kept from the vendor: a failed low-byte write is retried
 * with the HIGH byte. */
CyU3PReturnStatus_t sensor_set_gain(uint8_t cam, uint16_t percent)
{
    CyU3PReturnStatus_t status;
    uint16_t regval;
    float gain;

    status = CY_U3P_SUCCESS;
    gain = 0;
    gain = percent;
    gain = gain * SC_GAIN_UNITY;
    gain = gain / 100.0f;
    regval = (uint16_t)gain;  /* truncated, then rounded */
    if (gain - 0.5 > regval)
        regval = regval + 1;

    i2c_select_target(cam);
    if (cam == I2C_TARGET_CAM4) {
        status = sc130_reg_write(SC_REG_GAIN_HI, HIBYTE(regval));
        if (status != CY_U3P_SUCCESS) {
            CyU3PThreadSleep(2);
            status = sc130_reg_write(SC_REG_GAIN_HI, HIBYTE(regval));
            if (status != CY_U3P_SUCCESS) {
                CyU3PDebugPrint(4, "write seneor_register failed ret %d\r\n", status);
            }
        }
        status = sc130_reg_write(SC_REG_GAIN_LO, (uint8_t)regval);
        if (status != CY_U3P_SUCCESS) {
            CyU3PThreadSleep(2);
            status = sc130_reg_write(SC_REG_GAIN_LO, HIBYTE(regval));
            if (status != CY_U3P_SUCCESS) {
                CyU3PDebugPrint(4, "write seneor_register failed ret %d\r\n", status);
            }
        }
    } else {
        status = sensor_reg_write(SC_REG_GAIN_HI, HIBYTE(regval));
        if (status != CY_U3P_SUCCESS) {
            CyU3PThreadSleep(2);
            status = sc130_reg_write(SC_REG_GAIN_HI, HIBYTE(regval));
            if (status != CY_U3P_SUCCESS) {
                CyU3PDebugPrint(4, "write seneor_register failed ret %d\r\n", status);
            }
        }
        status = sensor_reg_write(SC_REG_GAIN_LO, (uint8_t)regval);
        if (status != CY_U3P_SUCCESS) {
            CyU3PThreadSleep(2);
            status = sc130_reg_write(SC_REG_GAIN_LO, HIBYTE(regval));
            if (status != CY_U3P_SUCCESS) {
                CyU3PDebugPrint(4, "write seneor_register failed ret %d\r\n", status);
            }
        }
    }
    i2c_select_target(I2C_TARGET_NONE);
    return status;
}

/* Gain readback from 0x3e08/0x3e09, converted back to percent. */
uint16_t sensor_get_gain(uint8_t cam)
{
    uint16_t regval;
    CyU3PReturnStatus_t status;
    float gain;
    uint8_t hi = 0;              /* defined result if the read fails */
    uint8_t lo = 0;
    CyU3PI2cPreamble_t preamble;

    status = CY_U3P_SUCCESS;
    gain = 0;

    i2c_lock();
    i2c_select_target(cam);
    preamble.length = 4;
    preamble.buffer[1] = SC_REG_GAIN_HI >> 8;
    preamble.buffer[2] = SC_REG_GAIN_HI & 0xff;
    preamble.buffer[0] = I2C_SENSOR_WR;
    preamble.buffer[3] = I2C_SENSOR_RD;
    preamble.ctrlMask = 0x0004;          /* repeated start before byte 3 */
    if (cam == I2C_TARGET_CAM4) {
        preamble.buffer[0] = I2C_SENSOR_WR;
        preamble.buffer[3] = I2C_SENSOR_RD;
    }
    status = CyU3PI2cReceiveBytes(&preamble, &hi, 1, 0);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "read seneor_register from 3e08 failed ret = %d\r\n", status);
        status = CyU3PI2cReceiveBytes(&preamble, &hi, 1, 0);
    }
    preamble.buffer[1] = SC_REG_GAIN_LO >> 8;
    preamble.buffer[2] = SC_REG_GAIN_LO & 0xff;
    preamble.ctrlMask = 0x0004;
    status = CyU3PI2cReceiveBytes(&preamble, &lo, 1, 0);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "read seneor_register from 3e09 failed ret = %d\r\n", status);
        status = CyU3PI2cReceiveBytes(&preamble, &lo, 1, 0);
    }
    regval = (hi << 8) | lo;
    gain = regval;
    gain = gain * 100.0f;
    gain = gain / SC_GAIN_UNITY;
    regval = (uint16_t)gain;  /* truncated, then rounded */
    if (gain - 0.5 > regval) {
        regval = regval + 1;
    }
    i2c_select_target(I2C_TARGET_NONE);
    i2c_unlock();
    return regval;
}

/* Write a register table (until register 0x0001) to one camera. */
CyU3PReturnStatus_t sensor_write_table(uint8_t cam, const sensor_reg_t *table)
{
    int i;
    CyU3PReturnStatus_t status;

    i = 0;
    status = CY_U3P_SUCCESS;
    i2c_select_target(cam);
    while (table[i].reg != SC_TABLE_END) {
        status = sc130_reg_write(table[i].reg, table[i].val);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(4, "sc130gs_write_table_for_sc130 write seneor_register addr:0x%x val:0x%x failed ret 0x%x\r\n", table[i].reg, table[i].val, status);
            i2c_select_target(I2C_TARGET_NONE);
            return status;
        }
        i = i + 1;
    }
    i2c_select_target(I2C_TARGET_NONE);
    return CY_U3P_SUCCESS;
}

CyU3PReturnStatus_t sensors_init(void)
{
    CyU3PReturnStatus_t status;

    status = sensor_write_table(I2C_TARGET_CAM1, sc130_init_regs);
    CyU3PDebugPrint(4, "sc130gs_write_table_for_sc130 GPIO_LVDS_CAM1, ErrCode:0x%x\n\r", status);
    status = sensor_write_table(I2C_TARGET_CAM2, sc130_init_regs);
    CyU3PDebugPrint(4, "sc130gs_write_table_for_sc130 GPIO_LVDS_CAM2, ErrCode:0x%x\n\r", status);
    status = sensor_write_table(I2C_TARGET_CAM4, sc130_init_regs);
    CyU3PDebugPrint(4, "sc130gs_write_table_for_sc130 GPIO_LVDS_CAM4:%d\n\r", status);
    return status;
}

/* FPGA register 10: laser mode in bits 4-6, status LED in bits 0-2 (2 = ok, 4 = fault, as used by
 * emc_wdg_thread). FPGA_KEEP for either field sets its "leave unchanged" flag (bit 7 / bit 3). */

int fpga_set_mode(uint8_t ld_mode, uint8_t led)
{
    struct fpga_word reg = {0};

    reg.b3 = ((ld_mode & FPGA_MODE_FIELD) << FPGA_MODE_LD_SHIFT) + (led & FPGA_MODE_FIELD);

    if (ld_mode == FPGA_KEEP)
        reg.b3 |= FPGA_MODE_LD_KEEP;

    if (led == FPGA_KEEP)
        reg.b3 |= FPGA_MODE_LED_KEEP;

    fpga_reg_write(FPGA_REG_MODE, &reg.b0);

    return 0;
}
