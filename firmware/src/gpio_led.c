/* gpio_led.c -- GPIO set-up, the I2C target select lines and the heartbeat output. */
#include "app.h"

uint8_t heartbeat_level = 0;           /* level driven next on the heartbeat GPIO */

CyU3PReturnStatus_t gpio_init(void)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PGpioClock_t gpioClock;
    CyU3PGpioSimpleConfig_t gpioConfig;

    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 2;
    gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc = CY_U3P_SYS_CLK;
    gpioClock.halfDiv = 0;
    status = CyU3PGpioInit(&gpioClock, gpio_interrupt_cb);
    if (status != CY_U3P_SUCCESS)
        CyU3PDebugPrint(4, "GPIO Init failed, Error Code = %d\r\n", status);

    gpioConfig.outValue = CyFalse;
    gpioConfig.driveLowEn = CyFalse;
    gpioConfig.driveHighEn = CyFalse;
    gpioConfig.inputEn = CyTrue;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;

    status = CyU3PDeviceGpioOverride(GPIO_FPGA_PROGRAM, CyTrue);
    status = CyU3PDeviceGpioOverride(GPIO_FPGA_IRQ, CyTrue);
    status = CyU3PDeviceGpioOverride(GPIO_IN_52, CyTrue);
    status = CyU3PDeviceGpioOverride(GPIO_FPGA_CFG_SEL, CyTrue);
    status = CyU3PDeviceGpioOverride(GPIO_I2C_SEL0, CyTrue);
    status = CyU3PDeviceGpioOverride(GPIO_I2C_SEL1, CyTrue);
    status = CyU3PDeviceGpioOverride(GPIO_I2C_SEL2, CyTrue);
    status = CyU3PDeviceGpioOverride(GPIO_I2C_SEL3, CyTrue);
    CyU3PDeviceGpioOverride(GPIO_HEARTBEAT, CyTrue);

    /* outputs, initially high */
    gpioConfig.driveLowEn = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.inputEn = CyFalse;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
    gpioConfig.outValue = CyTrue;
    status = CyU3PGpioSetSimpleConfig(GPIO_FPGA_PROGRAM, &gpioConfig);
    status = CyU3PGpioSetSimpleConfig(GPIO_FPGA_CFG_SEL, &gpioConfig);
    status = CyU3PGpioSetSimpleConfig(GPIO_I2C_SEL0, &gpioConfig);
    status = CyU3PGpioSetSimpleConfig(GPIO_I2C_SEL1, &gpioConfig);
    status = CyU3PGpioSetSimpleConfig(GPIO_I2C_SEL2, &gpioConfig);
    status = CyU3PGpioSetSimpleConfig(GPIO_I2C_SEL3, &gpioConfig);
    CyU3PGpioSetSimpleConfig(GPIO_HEARTBEAT, &gpioConfig);

    /* inputs */
    gpioConfig.outValue = CyFalse;
    gpioConfig.driveLowEn = CyFalse;
    gpioConfig.driveHighEn = CyFalse;
    gpioConfig.inputEn = CyTrue;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
    status = CyU3PGpioSetSimpleConfig(GPIO_IN_52, &gpioConfig);
    gpioConfig.intrMode = CY_U3P_GPIO_INTR_POS_EDGE;
    status = CyU3PGpioSetSimpleConfig(GPIO_FPGA_IRQ, &gpioConfig);
    return status;
}

/* Put the I2C target code (I2C_TARGET_*) on the 4 select lines, bit 0 on GPIO 23 ... bit 3 on
 * GPIO 27. Codes above 9 deselect. 3 ms settle time before and after. */
CyU3PReturnStatus_t i2c_select_target(uint8_t target)
{
    CyU3PReturnStatus_t status;

    CyU3PThreadSleep(3);
    switch (target) {
    case 0:
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL0, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL1, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL2, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL3, CyFalse);
        break;
    case 1:
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL0, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL1, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL2, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL3, CyFalse);
        break;
    case 2:
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL0, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL1, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL2, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL3, CyFalse);
        break;
    case 3:
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL0, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL1, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL2, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL3, CyFalse);
        break;
    case 4:
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL0, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL1, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL2, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL3, CyFalse);
        break;
    case 5:
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL0, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL1, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL2, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL3, CyFalse);
        break;
    case 6:
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL0, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL1, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL2, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL3, CyFalse);
        break;
    case 7:
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL0, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL1, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL2, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL3, CyFalse);
        break;
    case 8:
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL0, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL1, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL2, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL3, CyTrue);
        break;
    case 9:
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL0, CyTrue);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL1, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL2, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL3, CyTrue);
        break;
    default:
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL0, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL1, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL2, CyFalse);
        status = CyU3PGpioSimpleSetValue(GPIO_I2C_SEL3, CyFalse);
        break;
    }
    CyU3PThreadSleep(3);
    return status;
}

/* Called once a second by emc_wdg_thread(). */
void heartbeat_toggle(void)
{
    CyU3PGpioSimpleSetValue(GPIO_HEARTBEAT, heartbeat_level);
    heartbeat_level = ~heartbeat_level;
}
