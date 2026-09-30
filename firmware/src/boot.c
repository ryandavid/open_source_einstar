/* boot.c -- EEPROM boot signature, debug console and GPIF II start-up. */
#include "app.h"

#define EEPROM_I2C_ADDR   0xA0          /* FX3 boot EEPROM (holds the first-stage bootloader) */

/* Zero the first two bytes of the boot EEPROM, i.e. the "CY" signature, so the FX3 boot ROM no
 * longer boots from it and falls back to its USB boot loader (HID SET_REPORT 0xAA, usb_setup_cb). */
void eeprom_invalidate_cy(void)
{
    CyU3PI2cPreamble_t preamble;
    uint8_t data[2];

    data[0] = 0;
    data[1] = 0;
    preamble.buffer[0] = EEPROM_I2C_ADDR;   /* write */
    preamble.buffer[1] = 0;                 /* offset 0x0000 */
    preamble.buffer[2] = 0;
    preamble.length = 3;
    preamble.ctrlMask = 0;
    CyU3PI2cTransmitBytes(&preamble, data, 2, 0);
}

/* Debug console on the UART, set up as in the SDK examples. Failures are fatal. */
void debug_console_init(void)
{
    CyU3PReturnStatus_t status;
    CyU3PUartConfig_t uartConfig;

    status = CY_U3P_SUCCESS;

    status = CyU3PUartInit();
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "Error Code = %d\r\n", status);
        for (;;) ;
    }

    CyU3PMemSet((uint8_t *)&uartConfig, 0, sizeof(uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit  = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity   = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma    = CyTrue;

    status = CyU3PUartSetConfig(&uartConfig, NULL);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "Error Code = %d\r\n", status);
        for (;;) ;
    }

    status = CyU3PUartTxSetBlockXfer(0xFFFFFFFF);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "Error Code = %d\r\n", status);
        for (;;) ;
    }

    status = CyU3PDebugInit(CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "Error Code = %d\r\n", status);
        for (;;) ;
    }

    CyU3PDebugPreamble(CyFalse);
}

/* Load the GPIF II configuration (gpif_config.c) and start the state machine. GPIF threads 0 and 3
 * feed PIB sockets 0 and 3 (image data towards EP 0x83). */
#define GPIF_START_STATE   0
#define GPIF_ALPHA_START   12

void gpif_start(void)
{
    CyU3PReturnStatus_t status;

    status = CyU3PGpifLoad(&CyFxGpifConfig);
    if (status != CY_U3P_SUCCESS)
        CyU3PDebugPrint(4, "CyU3PGpifLoad failed, Error Code = %d\n", status);

    CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_0, 3, CyFalse, 1);
    CyU3PGpifSocketConfigure(3, CY_U3P_PIB_SOCKET_3, 3, CyFalse, 1);

    status = CyU3PGpifSMStart(GPIF_START_STATE, GPIF_ALPHA_START);
    if (status != CY_U3P_SUCCESS)
        CyU3PDebugPrint(4, "CyU3PGpifSMStart failed, Error Code = %d\n", status);
}
