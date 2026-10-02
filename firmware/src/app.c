/* app.c -- the four threads, CyFxApplicationDefine() and main(). */
#include "app.h"

/* Flash status-register writes done at every boot (Winbond-style W25Q commands): clear the
 * block-protect bits and set QE. The flash is left unprotected. */
#define FLASH_WRSR1    0x01
#define FLASH_WRSR2    0x31
#define FLASH_WRSR3    0x11
#define FLASH_SR1_NO_PROTECT 0x00   /* BP0-2, TB, SEC, SRP0 cleared */
#define FLASH_SR2_QE   0x02
#define FLASH_SR3_DEFAULT 0x00

struct flash_sr { uint8_t sr1, sr2, sr3, pad; };   /* flash_read_status_regs() result */

/* Threads: priorities, preemption threshold, stacks */
#define PRIO_FPGA_BOOT       9
#define PRIO_WORKER          8
#define PREEMPT_THRESHOLD    8
#define THREAD_STACK_SIZE    4096
#define WDG_STACK_SIZE       1024

#define EMC_WDG_PERIOD_MS    1000       /* emc_wdg_timer (never started) */
#define USB_CONFIG_TIMEOUT_S 10         /* powered but unconfigured: reconnect + reboot */
#define CMD_REPLY_BUF_SIZE   50         /* 00/05's reply is 51 bytes (9 + 42) */
#define FPGA_LD_MODE_BOOT    2          /* laser mode set at start-up (as 10/62 DISTANCE 2) */

/* "20:FPGA Boot Thread" (auto-started, priority 9): peripherals, flash status registers, FPGA
 * configuration, then starts the other two worker threads. */
void fpga_boot_thread(uint32_t input)
{
    (void)input;  /* ThreadX entry argument, unused */
    struct flash_sr sr = {0};

    spi_init();
    gpio_init();
    io_matrix_init();
    debug_console_init();
    CyU3PDebugPrint(4, "flash_read_status\r\n");

    flash_read_status_regs((uint8_t *)&sr);
    CyU3PDebugPrint(4, "before write SR:[0x%x 0x%x 0x%x]\r\n", sr.sr1, sr.sr2, sr.sr3);

    flash_write_status_reg(FLASH_WRSR1, FLASH_SR1_NO_PROTECT, 0);
    flash_write_status_reg(FLASH_WRSR2, FLASH_SR2_QE, 0);
    flash_write_status_reg(FLASH_WRSR3, FLASH_SR3_DEFAULT, 0);

    flash_read_status_regs((uint8_t *)&sr);
    CyU3PDebugPrint(4, "after write SR:[0x%x 0x%x 0x%x]\r\n", sr.sr1, sr.sr2, sr.sr3);

    i2c_init();
    gpio_init();
    fpga_load();
    CyU3PThreadSleep(500);
    CyU3PThreadResume(&com_thread_obj);
    CyU3PThreadResume(&control_thread_obj);
}

/* "21:comThread": camera and USB bring-up, the watchdog timers, then connects to USB and serves
 * the bulk channel (EP 0x02 -> bulk_dispatch() -> EP 0x82). */
void com_thread(uint32_t input)
{
    (void)input;  /* ThreadX entry argument, unused */
    CyU3PReturnStatus_t status;
    uint16_t replyLength;
    CyU3PDmaBuffer_t inBuf;
    CyU3PDmaBuffer_t req;
    CyU3PDmaBuffer_t outBuf;

    status = CY_U3P_SUCCESS;

    CyU3PDebugPrint(4, "Config cameras...\r\n");
    sensors_init();
    sc132_set_gain_linear_mode();
    light_init();
    usb_init();

    CyU3PDebugPrint(2, "dscr Init Successful\r\n");
    gpif_start();
    build_version_string();

    CyU3PDebugPrint(2, "EMC_WDG_ProgressTimer Successful1\r\n");

    CyU3PTimerCreate(&emc_wdg_timer, emc_wdg_timer_cb, 0, EMC_WDG_PERIOD_MS, EMC_WDG_PERIOD_MS, CYU3P_NO_ACTIVATE);
    CyU3PTimerCreate(&update_timer, update_timer_cb, 0, UPDATE_TIMEOUT_MS, UPDATE_TIMEOUT_MS, CYU3P_NO_ACTIVATE);

    fpga_set_mode(FPGA_LD_MODE_BOOT, FPGA_LED_OK);
    CyU3PConnectState(CyTrue, CyTrue);
    CyU3PThreadSleep(3000);

    for (;;) {
        if (app_active != 0) {
            status = CyU3PDmaChannelGetBuffer(&ch_bulk_out, &inBuf, CYU3P_WAIT_FOREVER);
            if (status != CY_U3P_SUCCESS) {
                if (app_active == 0)
                    continue;
                CyU3PDebugPrint(4, "CyU3PDmaChannelGetBuffer failed, Error code = %d\r\n", status);
            }

            req = inBuf;

            status = CyU3PDmaChannelDiscardBuffer(&ch_bulk_out);
            if (status != CY_U3P_SUCCESS) {
                CyU3PDebugPrint(4, "Bulk DMA Discard failed!\r\n");
            }

            status = CyU3PDmaChannelGetBuffer(&ch_bulk_in, &outBuf, CYU3P_WAIT_FOREVER);

            replyLength = (uint16_t)bulk_dispatch(req.buffer, outBuf.buffer);
            status = CyU3PDmaChannelCommitBuffer(&ch_bulk_in, replyLength, 0);
            if (status != CY_U3P_SUCCESS) {
                CyU3PUsbSetEpNak(EP_BULK_IN, CyTrue);
                CyU3PThreadSleep(1);
                CyU3PDmaChannelReset(&ch_bulk_in);
                CyU3PDmaChannelSetXfer(&ch_bulk_in, 0);
                CyU3PThreadSleep(25);
                CyU3PUsbFlushEp(EP_BULK_IN);
                CyU3PUsbSetEpNak(EP_BULK_IN, CyFalse);
                CyU3PThreadSleep(1);
                CyU3PDebugPrint(4, "Bulk DMA send failed!\r\n");
            }
        }

        CyU3PThreadRelinquish();
    }
}

/* "21:EMC_WDG_Thread" (auto-started): once a second toggles the heartbeat GPIO and reboots the
 * device when an endpoint was reset (usb_ep_halted), an update completed, or USB has been powered
 * but not configured for 10 s. The progress check (emc_wdg_progress > emc_wdg_limit) is inert:
 * emc_wdg_timer is never started, so the count stays 0. */
void emc_wdg_thread(uint32_t input)
{
    (void)input;  /* ThreadX entry argument, unused */
    CyBool_t connected;

    connected = 0;

    for (;;) {
        CyU3PThreadSleep(1000);
        heartbeat_toggle();

        if (emc_wdg_progress > emc_wdg_limit) {
            fpga_set_mode(FPGA_KEEP, FPGA_LED_FAULT);
            CyU3PDebugPrint(4, "EMC WDG Timeout %d\r\n", emc_wdg_progress);
            CyU3PThreadSleep(300);
            CyU3PDeviceReset(CyFalse);
        } else {
            fpga_set_mode(FPGA_KEEP, FPGA_LED_OK);
        }

        if (usb_ep_halted != 0) {
            CyU3PDebugPrint(4, "Detect USB Transfer Error!!! Reboot device\r\n");
            CyU3PDeviceReset(CyFalse);
        }

        if (app_state.update_done == 1) {
            app_state.update_done = 0;
            CyU3PThreadSleep(25);
            CyU3PDeviceReset(CyFalse);
        }

        connected = CyU3PGetConnectState();

        if (connected == 1 && app_state.usb_powered == 1 && app_state.usb_configured == 1) {
            usb_unconfigured_secs = 0;
        } else if (app_state.usb_powered == 0) {
            usb_unconfigured_secs = 0;
        } else {
            fpga_set_mode(FPGA_KEEP, FPGA_LED_FAULT);
            usb_unconfigured_secs = usb_unconfigured_secs + 1;
        }

        if (usb_unconfigured_secs > USB_CONFIG_TIMEOUT_S - 1) {
            CyU3PDebugPrint(4, "\r\nU_u USB Reconnect...\r\n");
            usb_unconfigured_secs = 0;
            CyU3PConnectState(CyFalse, CyFalse);
            CyU3PThreadSleep(25);
            CyU3PDeviceReset(CyFalse);
        }
    }
}

/* "21:ControlThread": the command channel. Each command from EP 0x01 goes to command_dispatch();
 * its reply (in a 50-byte buffer) goes out on EP 0x81. Command 8 asks for a reboot after the reply. */
void control_thread(uint32_t input)
{
    (void)input;  /* ThreadX entry argument, unused */
    CyU3PReturnStatus_t status;
    uint8_t *reply;
    uint16_t replyLength;
    CyU3PDmaBuffer_t inBuf;
    CyU3PDmaBuffer_t cmd;

    status = CY_U3P_SUCCESS;
    reply = NULL;

top:
    if (app_active != 0) {
        status = CyU3PDmaChannelGetBuffer(&ch_ctrl_out, &inBuf, CYU3P_WAIT_FOREVER);
        if (status != CY_U3P_SUCCESS) {
            if (app_active == 0)
                goto next;
            CyU3PDebugPrint(4, "CyU3PDmaChannelGetBuffer failed, Error code = %d\r\n", status);
        }

        cmd = inBuf;

        status = CyU3PDmaChannelDiscardBuffer(&ch_ctrl_out);
        if (status != CY_U3P_SUCCESS)
            CyU3PDebugPrint(4, "Control DMA Discard failed!\r\n");

        reply = CyU3PMemAlloc(CMD_REPLY_BUF_SIZE);
        if (reply == NULL)
            goto next;

        replyLength = (uint16_t)command_dispatch(cmd, reply);
        if (replyLength != 0) {
            status = CyU3PDmaChannelSendData(&ch_ctrl_in, reply, replyLength);
        }

        if (status != CY_U3P_SUCCESS) {
            CyU3PUsbSetEpNak(EP_CTRL_IN, CyTrue);
            CyU3PThreadSleep(1);
            CyU3PDmaChannelReset(&ch_ctrl_in);
            CyU3PDmaChannelSetXfer(&ch_ctrl_in, 0);
            CyU3PThreadSleep(25);
            CyU3PUsbFlushEp(EP_CTRL_IN);
            CyU3PUsbSetEpNak(EP_CTRL_IN, CyFalse);
            CyU3PThreadSleep(1);
            CyU3PDebugPrint(4, "Control DMA send failed!\r\n");
        }

        CyU3PMemFree(reply);

        if (app_state.reboot_after_reply == 1) {
            app_state.reboot_after_reply = 0;
            CyU3PThreadSleep(25);
            CyU3PDeviceReset(CyFalse);
        }
    }

    CyU3PThreadRelinquish();
    goto top;

next:
    goto top;
}

/* Threads. Only the FPGA boot and watchdog threads start immediately; the FPGA boot thread resumes
 * the other two. Note: the watchdog thread's creation result is not checked, its stack is not
 * NULL-checked, and flash_sem is never created (only i2c_sem), so flash_lock() does not lock. */
void CyFxApplicationDefine(void)
{
    void *stack1;
    void *stack2;
    void *stack3;
    void *stack4;
    CyU3PReturnStatus_t status;

    stack1 = CyU3PMemAlloc(THREAD_STACK_SIZE);
    stack2 = CyU3PMemAlloc(THREAD_STACK_SIZE);
    stack3 = CyU3PMemAlloc(THREAD_STACK_SIZE);
    stack4 = CyU3PMemAlloc(WDG_STACK_SIZE);

    if (stack1 == NULL || stack2 == NULL || stack3 == NULL)
    {
        goto error;
    }

    status = CyU3PThreadCreate(&fpga_boot_thread_obj, "20:FPGA Boot Thread", &fpga_boot_thread, 0,
                               stack1, THREAD_STACK_SIZE, PRIO_FPGA_BOOT, PREEMPT_THRESHOLD, CYU3P_NO_TIME_SLICE, CYU3P_AUTO_START);
    if (status != CY_U3P_SUCCESS)
    {
        goto error;
    }

    status = CyU3PThreadCreate(&com_thread_obj, "21:comThread", &com_thread, 0,
                               stack2, THREAD_STACK_SIZE, PRIO_WORKER, PREEMPT_THRESHOLD, CYU3P_NO_TIME_SLICE, CYU3P_DONT_START);
    if (status != CY_U3P_SUCCESS)
    {
        goto error;
    }

    status = CyU3PThreadCreate(&control_thread_obj, "21:ControlThread", &control_thread, 0,
                               stack3, THREAD_STACK_SIZE, PRIO_WORKER, PREEMPT_THRESHOLD, CYU3P_NO_TIME_SLICE, CYU3P_DONT_START);
    if (status != CY_U3P_SUCCESS)
    {
        goto error;
    }

    status = CyU3PThreadCreate(&emc_wdg_thread_obj, "21:EMC_WDG_Thread", &emc_wdg_thread, 0,
                               stack4, WDG_STACK_SIZE, PRIO_WORKER, PREEMPT_THRESHOLD, CYU3P_NO_TIME_SLICE, CYU3P_AUTO_START);
    status = CyU3PSemaphoreCreate(&i2c_sem, 1);
    if (status != CY_U3P_SUCCESS)
    {
        goto error;
    }

    return;

error:
    for (;;) { }
}

int main(void)
{
    CyU3PReturnStatus_t status;
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PSysClockConfig_t clkCfg;

    clkCfg.setSysClk400 = CyTrue;
    clkCfg.cpuClkDiv = 2;
    clkCfg.dmaClkDiv = 2;
    clkCfg.mmioClkDiv = 2;
    clkCfg.useStandbyClk = CyFalse;
    clkCfg.clkSrc = CY_U3P_SYS_CLK;
    status = CyU3PDeviceInit(&clkCfg);
    if (status != CY_U3P_SUCCESS)
        goto handle_fatal_error;

    status = CyU3PDeviceCacheControl(CyTrue, CyFalse, CyFalse);

    /* no UART yet: io_matrix_init() adds it once the FPGA boot thread runs */
    CyU3PMemSet((uint8_t *)&io_cfg, 0, sizeof(io_cfg));   /* s0Mode/s1Mode: leave the S-ports off */
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    io_cfg.gpioSimpleEn[0] = 0;
    io_cfg.gpioSimpleEn[1] = GPIO_SIMPLE_EN_HI;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    io_cfg.useUart = CyFalse;
    io_cfg.useI2C = CyTrue;
    io_cfg.useI2S = CyFalse;
    io_cfg.useSpi = CyTrue;
    status = CyU3PDeviceConfigureIOMatrix(&io_cfg);
    if (status != CY_U3P_SUCCESS)
        goto handle_fatal_error;

    CyU3PKernelEntry();
    return 0;

handle_fatal_error:
    for (;;) {}
}
