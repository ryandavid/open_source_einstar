/* usb.c -- USB: endpoint and DMA-channel setup, USB event and control-request callbacks, USB and
 * I2C/IO-matrix initialisation. Structure follows the SDK bulk-loop example. */
#include "app.h"

#define DMA_BUF_COUNT        4
#define BULK_DMA_BUF_SIZE    5120
#define IMAGE_DMA_BUF_SIZE   0xA020

/* Endpoints and DMA channels. The speed is fixed at high speed: all endpoints use 512-byte
 * packets and burst length 1, also on SuperSpeed links. */
void app_start(void)
{
    uint16_t size;
    CyU3PReturnStatus_t status;
    CyU3PUSBSpeed_t usbSpeed;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;

    size = 0;
    status = CY_U3P_SUCCESS;
    usbSpeed = CY_U3P_HIGH_SPEED;

    switch (usbSpeed) {
    case CY_U3P_FULL_SPEED:
        size = 64;
        break;
    case CY_U3P_HIGH_SPEED:
        size = 512;
        break;
    case CY_U3P_SUPER_SPEED:
        size = 1024;
        break;
    case CY_U3P_NOT_CONNECTED:
    default:
        CyU3PDebugPrint(4, "Error! Invalid USB speed.\n");
        break;
    }

    /* EP 0x01 / 0x81: command channel (CPU <-> USB, manual DMA) */
    CyU3PMemSet((uint8_t *)&epCfg, 0, sizeof(epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_INTR;
    epCfg.burstLen = (usbSpeed == CY_U3P_SUPER_SPEED) ? 16 : 1;
    epCfg.isoPkts = 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;
    status = CyU3PSetEpConfig(EP_CTRL_OUT, &epCfg);
    if (status) CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", status);
    status = CyU3PSetEpConfig(EP_CTRL_IN, &epCfg);
    if (status) CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", status);
    CyU3PUsbFlushEp(EP_CTRL_OUT);
    CyU3PUsbFlushEp(EP_CTRL_IN);

    dmaCfg.size = size;
    dmaCfg.count = DMA_BUF_COUNT;
    dmaCfg.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaCfg.consSckId = CY_U3P_UIB_SOCKET_CONS_1;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;
    status = CyU3PDmaChannelCreate(&ch_ctrl_in, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaCfg);
    if (status) CyU3PDebugPrint(4, "CyU3PDmaChanneloutCreate failed, Error code = %d\n", status);
    status = CyU3PDmaChannelSetXfer(&ch_ctrl_in, 0);
    if (status) CyU3PDebugPrint(4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", status);

    dmaCfg.size = size;
    dmaCfg.count = DMA_BUF_COUNT;
    dmaCfg.prodSckId = CY_U3P_UIB_SOCKET_PROD_1;
    dmaCfg.consSckId = CY_U3P_CPU_SOCKET_CONS;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = CY_U3P_DMA_CB_PROD_EVENT;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;
    status = CyU3PDmaChannelCreate(&ch_ctrl_out, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaCfg);
    if (status) CyU3PDebugPrint(4, "CyU3PDmaChannelinCreate failed, Error code = %d\n", status);
    status = CyU3PDmaChannelSetXfer(&ch_ctrl_out, 0);
    if (status) CyU3PDebugPrint(4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", status);

    /* EP 0x02 / 0x82: bulk channel (update, user pages) */
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = (usbSpeed == CY_U3P_SUPER_SPEED) ? 16 : 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;
    epCfg.isoPkts = 1;
    status = CyU3PSetEpConfig(EP_BULK_OUT, &epCfg);
    if (status) CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", status);
    status = CyU3PSetEpConfig(EP_BULK_IN, &epCfg);
    if (status) CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", status);
    CyU3PUsbFlushEp(EP_BULK_OUT);
    CyU3PUsbFlushEp(EP_BULK_IN);

    dmaCfg.size = BULK_DMA_BUF_SIZE;
    dmaCfg.count = DMA_BUF_COUNT;
    dmaCfg.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaCfg.consSckId = CY_U3P_UIB_SOCKET_CONS_2;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;
    status = CyU3PDmaChannelCreate(&ch_bulk_in, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaCfg);
    if (status) CyU3PDebugPrint(4, "CyU3PDmaChannelBulkoutCreate failed, Error code = %d\n", status);
    status = CyU3PDmaChannelSetXfer(&ch_bulk_in, 0);
    if (status) CyU3PDebugPrint(4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", status);

    dmaCfg.size = BULK_DMA_BUF_SIZE;
    dmaCfg.count = DMA_BUF_COUNT;
    dmaCfg.prodSckId = CY_U3P_UIB_SOCKET_PROD_2;
    dmaCfg.consSckId = CY_U3P_CPU_SOCKET_CONS;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;
    status = CyU3PDmaChannelCreate(&ch_bulk_out, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaCfg);
    if (status) CyU3PDebugPrint(4, "CyU3PDmaChannelBulkinCreate failed, Error code = %d\n", status);
    status = CyU3PDmaChannelSetXfer(&ch_bulk_out, 0);
    if (status) CyU3PDebugPrint(4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", status);

    /* EP 0x83: image stream, GPIF II (PIB socket 0) -> USB, auto DMA */
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = (usbSpeed == CY_U3P_SUPER_SPEED) ? 16 : 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;
    epCfg.isoPkts = 1;
    status = CyU3PSetEpConfig(EP_IMAGE_IN, &epCfg);
    if (status) CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", status);
    CyU3PUsbFlushEp(EP_IMAGE_IN);

    dmaCfg.size = IMAGE_DMA_BUF_SIZE;
    dmaCfg.count = DMA_BUF_COUNT;
    dmaCfg.prodSckId = CY_U3P_PIB_SOCKET_0;
    dmaCfg.consSckId = CY_U3P_UIB_SOCKET_CONS_3;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.prodAvailCount = 0;
    status = CyU3PDmaChannelCreate(&ch_image, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (status) CyU3PDebugPrint(4, "CyU3PDmaChannelStreaminCreate failed, Error code = %d\n", status);
    status = CyU3PDmaChannelSetXfer(&ch_image, 0);
    if (status) CyU3PDebugPrint(4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", status);

    app_active = 1;
}

/* Tear down the channels and disable the endpoints. */
void app_stop(void)
{
    CyU3PReturnStatus_t status;
    CyU3PEpConfig_t epCfg;

    status = CY_U3P_SUCCESS;
    CyU3PDebugPrint(4, "STOP!!!!\n");
    app_active = 0;

    CyU3PDmaChannelDestroy(&ch_ctrl_out);
    CyU3PDmaChannelDestroy(&ch_ctrl_in);
    CyU3PDmaChannelDestroy(&ch_bulk_out);
    CyU3PDmaChannelDestroy(&ch_bulk_in);
    CyU3PDmaChannelDestroy(&ch_image);

    CyU3PUsbFlushEp(EP_CTRL_OUT);
    CyU3PUsbFlushEp(EP_CTRL_IN);
    CyU3PUsbFlushEp(EP_BULK_OUT);
    CyU3PUsbFlushEp(EP_BULK_IN);
    CyU3PUsbFlushEp(EP_IMAGE_IN);

    CyU3PMemSet((uint8_t *)&epCfg, 0, sizeof(epCfg));
    epCfg.enable = CyFalse;

    status = CyU3PSetEpConfig(EP_CTRL_OUT, &epCfg);
    if (status != CY_U3P_SUCCESS)
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", status);
    status = CyU3PSetEpConfig(EP_CTRL_IN, &epCfg);
    if (status != CY_U3P_SUCCESS)
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", status);
    status = CyU3PSetEpConfig(EP_BULK_OUT, &epCfg);
    if (status != CY_U3P_SUCCESS)
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", status);
    status = CyU3PSetEpConfig(EP_BULK_IN, &epCfg);
    if (status != CY_U3P_SUCCESS)
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", status);
    status = CyU3PSetEpConfig(EP_IMAGE_IN, &epCfg);
    if (status != CY_U3P_SUCCESS)
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", status);

    CyU3PGpifDisable(CyTrue);
}

/* Accept every U1/U2 (link power management) request. */
CyBool_t usb_lpm_request_cb(CyU3PUsbLinkPowerMode link_mode)
{
    (void)link_mode;
    return CyTrue;
}

/* Restart the image path: GPIF off, flush and reset EP 0x83 and its channel, GPIF on. */
void stream_reset(void)
{
    CyU3PGpifDisable(CyTrue);
    CyU3PDmaChannelReset(&ch_image);
    CyU3PUsbFlushEp(EP_IMAGE_IN);
    CyU3PUsbResetEp(EP_IMAGE_IN);
    CyU3PDmaChannelSetXfer(&ch_image, 0);
    gpif_start();
}

/* Full restart (command 7 after a CLEAR_FEATURE on EP 0x83): USB, FPGA, sensors, streaming. */
void app_restart(void)
{
    app_stop();
    usb_init();
    CyU3PThreadSleep(10);
    fpga_load();
    CyU3PThreadSleep(300);
    sensors_init();
    sc132_set_gain_linear_mode();
    app_start();
    stream_reset();
    CyU3PConnectState(CyTrue, CyTrue);
}

void usb_event_cb(CyU3PUsbEventType_t evtype, uint16_t evdata)
{
    CyU3PReturnStatus_t status;

    CyU3PDebugPrint(4, "Get Event! evtype=0x%x evdata=0x%x \r\n", evtype, evdata);
    status = CY_U3P_SUCCESS;

    switch (evtype) {
    case CY_U3P_USB_EVENT_RESET:
        app_state.usb_configured = 0;
        CyU3PDebugPrint(4, "USB reset...\r\n");
        break;
    case CY_U3P_USB_EVENT_DISCONNECT:
        CyU3PDebugPrint(4, "USB disconnected...\r\n");
        if (app_active != 0) {
            app_active = 0;
        }
        break;
    case CY_U3P_USB_EVENT_SUSPEND:
        app_state.usb_configured = 0;
        CyU3PDebugPrint(4, "USB SUSPEND...\r\n");
        break;
    case CY_U3P_USB_EVENT_CONNECT:
        CyU3PDebugPrint(4, "USB connected...\r\n");
        CyU3PUsbLPMDisable();
        break;
    case CY_U3P_USB_EVENT_SETCONF:
        app_state.usb_configured = 1;
        if (app_active != 0) {
            app_stop();
            app_active = 0;
        }
        CyU3PDebugPrint(4, "USB SETCONF.\r\n");
        CyU3PUsbLPMDisable();
        app_start();
        stream_reset();
        break;
    case CY_U3P_USB_EVENT_SPEED:
        CyU3PDebugPrint(4, "USB Speed Change.\r\n");
        break;
    case CY_U3P_USB_EVENT_SET_SEL:
        {
            int sel[6] = { 0, 0, 0, 0, 0, 0 };
            CyU3PUsbGetDevProperty(CY_U3P_USB_PROP_SYS_EXIT_LAT, (uint32_t *)sel);
            CyU3PDebugPrint(4, "latency Exit: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \r\n", sel[0], sel[1], sel[2], sel[3], sel[4], sel[5]);
        }
        break;
    case CY_U3P_USB_EVENT_VBUS_VALID:
        app_state.usb_powered = 1;
        CyU3PDebugPrint(4, "USB Powered\r\n");
        CyU3PThreadSleep(100);
        break;
    case CY_U3P_USB_EVENT_VBUS_REMOVED:
        CyU3PDebugPrint(4, "USB plug out\r\n");
        app_state.usb_powered = 0;
        CyU3PDeviceReset(CyFalse);
        break;
    case CY_U3P_USB_EVENT_USB3_LNKFAIL:
        CyU3PDebugPrint(4, "USB3_LNKFAIL event,USB plug out?\r\n");
        status = CyU3PConnectState(CyFalse, CyFalse);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(4, "USB Connect failed, Error code = %d\n", status);
        }
        status = CyU3PConnectState(CyTrue, CyTrue);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(4, "USB Connect failed, Error code = %d\n", status);
        }
        break;
    case CY_U3P_USB_EVENT_EP_UNDERRUN:
    case CY_U3P_USB_EVENT_EP0_STAT_CPLT:
    case CY_U3P_USB_EVENT_HOST_CONNECT:
    case CY_U3P_USB_EVENT_HOST_DISCONNECT:
    case CY_U3P_USB_EVENT_LMP_EXCH_FAIL:
    case CY_U3P_USB_EVENT_LNK_RECOVERY:
    case CY_U3P_USB_EVENT_OTG_CHANGE:
    case CY_U3P_USB_EVENT_OTG_SRP:
    case CY_U3P_USB_EVENT_OTG_VBUS_CHG:
    case CY_U3P_USB_EVENT_RESERVED_1:
    case CY_U3P_USB_EVENT_RESUME:
    case CY_U3P_USB_EVENT_SETINTF:
    case CY_U3P_USB_EVENT_SOF_ITP:
    case CY_U3P_USB_EVENT_SS_COMP_ENTRY:
    case CY_U3P_USB_EVENT_SS_COMP_EXIT:
    default:
        CyU3PDebugPrint(4, "Get Event! evtype=0x%x evdata=0x%x\r\n", evtype, evdata);
        break;
    }
}

/* Endpoint events: logs all but NAK/short packets; a SuperSpeed endpoint reset marks the USB
 * path as broken and emc_wdg_thread() reboots the device. */
void usb_ep_event_cb(CyU3PUsbEpEvtType evType, CyU3PUSBSpeed_t usbSpeed, uint8_t epNum)
{
    (void)usbSpeed;
    if (evType == CYU3P_USBEP_NAK_EVT || evType == CYU3P_USBEP_SLP_EVT)
        return;
    CyU3PDebugPrint(2, "EP Event CB epNum : %d evtype : %d\r\n", epNum, evType);
    if (evType == CYU3P_USBEP_SS_RESET_EVT) {
        CyU3PDebugPrint(2, "Halting EP epNum : %d\r\n", epNum);
        usb_ep_halted = 1;
    }
}

/* Control requests. Standard endpoint CLEAR_FEATURE resets the endpoint's DMA channel (EP 0x83
 * also restarts GPIF and arms command 7). Class requests to the device with wIndex 2 use HID's
 * request numbers: GET_REPORT returns hid_get_report_data, SET_REPORT with byte 1 = 0xAA erases
 * the EEPROM boot signature (0xBB only logs "Reset!"), GET/SET_IDLE keep a byte per report ID.
 * The "Class Request ..." / "Vendor Request" messages are the vendor's labels. */
#define RQT_STD_DEVICE       0x00      /* bmRequestType */
#define RQT_STD_INTERFACE    0x01
#define RQT_STD_ENDPOINT     0x02
#define RQT_CLASS_DEVICE     0x20
#define RQT_CLASS_INTERFACE  0x21
#define RQT_CLASS_ENDPOINT   0x22
#define REQ_CHANNEL          2         /* wIndex of the class requests */
#define REQ_GET_REPORT       1         /* bRequest, HID numbering */
#define REQ_GET_IDLE         2
#define REQ_GET_PROTOCOL     3
#define REQ_SET_REPORT       9
#define REQ_SET_IDLE         10
#define REQ_SET_PROTOCOL     11
#define REPORT_ID_2          2
#define REPORT_ID_7          7
#define SET_REPORT_EEPROM_ERASE  0xAA
#define SET_REPORT_RESET         0xBB
#define EP0_BUF_SIZE         64

/* Two structural details matter for identical code: the CLEAR_FEATURE endpoint switch ends in an
 * explicit `default: break;`, and SET_IDLE's if/else-if ends in an empty `else {}` (gcc 4.8 -O0
 * keeps the resulting jumps; without them it threads them past the break). */
CyBool_t usb_setup_cb(uint32_t setupdat0, uint32_t setupdat1)
{
    CyBool_t isHandled;
    unsigned char bmRequestType;
    unsigned char bRequest;
    unsigned short wValue;
    unsigned short wIndex;
    unsigned short wLength;
    CyU3PReturnStatus_t status;
    uint8_t buf[EP0_BUF_SIZE];
    uint16_t readCount;

    isHandled = CyFalse;
    bmRequestType = (unsigned char)setupdat0;
    bRequest = (setupdat0 & 0xff00) >> 8;
    wValue = (unsigned short)(setupdat0 >> 16);
    wIndex = (unsigned short)setupdat1;
    wLength = (unsigned short)(setupdat1 >> 16);

    CyU3PDebugPrint(4, "REQ 0x%x 0x%x 0x%x 0x%x 0x%x\n", bmRequestType, bRequest, wValue, wIndex, wLength);

    switch (bmRequestType) {
    case RQT_STD_DEVICE:
        CyU3PDebugPrint(4, "Class Request Target Device\r\n");
        break;

    case RQT_STD_INTERFACE:
        if (bRequest == CY_U3P_USB_SC_SET_FEATURE || bRequest == CY_U3P_USB_SC_CLEAR_FEATURE) {
            if (wValue == 0) {
                if (app_active != 0) {
                    CyU3PUsbAckSetup();
                } else {
                    CyU3PUsbStall(0, CyTrue, CyFalse);
                }
                isHandled = CyTrue;
            }
        }
        break;

    case RQT_STD_ENDPOINT:
        switch (bRequest) {
        case CY_U3P_USB_SC_GET_STATUS:
            CyU3PDebugPrint(4, "GET_STATUS Request \r\n");
            break;

        case CY_U3P_USB_SC_CLEAR_FEATURE:
            CyU3PDebugPrint(4, "CLEAR_FEATURE Request \r\n");
            switch (wIndex) {
            case EP_CTRL_IN:
                CyU3PDebugPrint(4, "Reset endpoint1 IN\r\n");
                CyU3PUsbSetEpNak(EP_CTRL_IN, CyTrue);
                CyFx3BusyWait(125);
                CyU3PDmaChannelReset(&ch_ctrl_in);
                CyU3PUsbFlushEp(EP_CTRL_IN);
                CyU3PUsbResetEp(EP_CTRL_IN);
                CyU3PDmaChannelSetXfer(&ch_ctrl_in, 0);
                CyU3PUsbStall(EP_CTRL_IN, CyFalse, CyTrue);
                isHandled = CyTrue;
                CyU3PUsbAckSetup();
                CyU3PUsbSetEpNak(EP_CTRL_IN, CyFalse);
                CyU3PDebugPrint(4, "Reset endpoint1 IN OK\r\n");
                break;

            case EP_CTRL_OUT:
                CyU3PDebugPrint(4, "Reset endpoint1 OUT\r\n");
                CyU3PUsbSetEpNak(EP_CTRL_OUT, CyTrue);
                CyFx3BusyWait(125);
                CyU3PDmaChannelReset(&ch_ctrl_out);
                CyU3PUsbFlushEp(EP_CTRL_OUT);
                CyU3PUsbResetEp(EP_CTRL_OUT);
                CyU3PDmaChannelSetXfer(&ch_ctrl_out, 0);
                CyU3PUsbStall(EP_CTRL_OUT, CyFalse, CyTrue);
                isHandled = CyTrue;
                CyU3PUsbAckSetup();
                CyU3PUsbSetEpNak(EP_CTRL_OUT, CyFalse);
                CyU3PDebugPrint(4, "Reset endpoint1 OUT OK\r\n");
                break;

            case EP_BULK_IN:
                CyU3PDebugPrint(4, "Reset endpoint2 IN\r\n");
                CyU3PUsbSetEpNak(EP_BULK_IN, CyTrue);
                CyFx3BusyWait(125);
                CyU3PDmaChannelReset(&ch_bulk_in);
                CyU3PUsbFlushEp(EP_BULK_IN);
                CyU3PUsbResetEp(EP_BULK_IN);
                CyU3PDmaChannelSetXfer(&ch_bulk_in, 0);
                CyU3PUsbStall(EP_BULK_IN, CyFalse, CyTrue);
                isHandled = CyTrue;
                CyU3PUsbAckSetup();
                CyU3PUsbSetEpNak(EP_BULK_IN, CyFalse);
                CyU3PDebugPrint(4, "Reset endpoint2 IN OK\r\n");
                break;

            case EP_BULK_OUT:
                CyU3PDebugPrint(4, "Reset endpoint2 OUT\r\n");
                CyU3PUsbSetEpNak(EP_BULK_OUT, CyTrue);
                CyFx3BusyWait(125);
                CyU3PDmaChannelReset(&ch_bulk_out);
                CyU3PUsbFlushEp(EP_BULK_OUT);
                CyU3PUsbResetEp(EP_BULK_OUT);
                CyU3PDmaChannelSetXfer(&ch_bulk_out, 0);
                CyU3PUsbStall(EP_BULK_OUT, CyFalse, CyTrue);
                isHandled = CyTrue;
                CyU3PUsbAckSetup();
                CyU3PUsbSetEpNak(EP_BULK_OUT, CyFalse);
                CyU3PDebugPrint(4, "Reset endpoint2 OUT OK\r\n");
                break;

            case EP_IMAGE_IN:
                CyU3PDebugPrint(4, "Reset endpoint3 IN\r\n");
                CyU3PGpifDisable(CyTrue);
                CyU3PUsbSetEpNak(EP_IMAGE_IN, CyTrue);
                CyFx3BusyWait(125);
                CyU3PDmaChannelReset(&ch_image);
                CyU3PUsbFlushEp(EP_IMAGE_IN);
                CyU3PUsbResetEp(EP_IMAGE_IN);
                CyU3PDmaChannelSetXfer(&ch_image, 0);
                CyU3PUsbStall(EP_IMAGE_IN, CyFalse, CyTrue);
                isHandled = CyTrue;
                CyU3PUsbAckSetup();
                gpif_start();
                CyU3PUsbSetEpNak(EP_IMAGE_IN, CyFalse);
                CyU3PDebugPrint(4, "Reset endpoint3 IN OK\r\n");
                ep83_halt_cleared = 1;
                break;

            default:
                break;
            }
            break;

        case CY_U3P_USB_SC_SET_FEATURE:
            CyU3PDebugPrint(4, "SET_FEATURE Request \r\n");
            break;

        case CY_U3P_USB_SC_SET_ADDRESS:
            CyU3PDebugPrint(4, "SET_ADDRESS Request \r\n");
            break;

        case CY_U3P_USB_SC_GET_DESCRIPTOR:
            CyU3PDebugPrint(4, "GET_DESCRIPTOR Request \r\n");
            break;

        case CY_U3P_USB_SC_SET_DESCRIPTOR:
            CyU3PDebugPrint(4, "SET_DESCRIPTOR Request \r\n");
            break;

        case CY_U3P_USB_SC_GET_CONFIGURATION:
            CyU3PDebugPrint(4, "GET_CONFIGURATION Request \r\n");
            break;

        case CY_U3P_USB_SC_SET_CONFIGURATION:
            CyU3PDebugPrint(4, "SET_CONFIGURATION Request \r\n");
            break;

        case CY_U3P_USB_SC_GET_INTERFACE:
            CyU3PDebugPrint(4, "GET_INTERFACE Request \r\n");
            break;

        case CY_U3P_USB_SC_SET_INTERFACE:
            CyU3PDebugPrint(4, "SET_INTERFACE Request \r\n");
            break;

        case CY_U3P_USB_SC_SYNC_FRAME:
            CyU3PDebugPrint(4, "SYNCH_FRAME Request \r\n");
            break;

        case CY_U3P_USB_SC_SET_SEL:
            CyU3PDebugPrint(4, "SET_SEL Request \r\n");
            break;

        case CY_U3P_USB_SC_SET_ISOC_DELAY:
            CyU3PDebugPrint(4, "SET_ISOCH_DELAY Request \r\n");
            break;

        default:
            CyU3PDebugPrint(4, "UnKnown Request \r\n");
            break;
        }
        break;

    case RQT_CLASS_DEVICE:
        if (wIndex == REQ_CHANNEL) {
            switch (bRequest) {
            case REQ_GET_REPORT:
                CyU3PMemSet(buf, 0, EP0_BUF_SIZE);
                CyU3PMemCopy(buf, hid_get_report_data, hid_get_report_data[0]);
                status = CyU3PUsbSendEP0Data(EP0_BUF_SIZE, buf);
                break;

            case REQ_GET_IDLE:
                if ((wValue & 0xff) == REPORT_ID_2) {
                    buf[0] = hid_idle_rate_2;
                } else if ((wValue & 0xff) == REPORT_ID_7) {
                    buf[0] = hid_idle_rate_7;
                }
                status = CyU3PUsbSendEP0Data(1, buf);
                break;

            case REQ_GET_PROTOCOL:
                CyU3PUsbAckSetup();
                CyU3PDebugPrint(4, " set feature 03\n");
                break;

            case REQ_SET_REPORT:
                status = CyU3PUsbGetEP0Data(EP0_BUF_SIZE, buf, &readCount);
                if (status == CY_U3P_SUCCESS) {
                    if (wIndex == REQ_CHANNEL) {
                        if (bRequest == REQ_SET_REPORT) {
                            CyU3PUsbGetEP0Data(EP0_BUF_SIZE, buf, &readCount);
                            /* only byte 1 is used; copy just the buffer's size (the vendor copied all
                             * 64 bytes into this 8-byte buffer, overrunning it into ch_bulk_in). */
                            CyU3PMemCopy(hid_set_report_buf, buf, sizeof(hid_set_report_buf));
                            if (hid_set_report_buf[1] == SET_REPORT_EEPROM_ERASE) {
                                CyU3PDebugPrint(4, "EEPROM Erase!\n");
                                eeprom_invalidate_cy();
                            } else if (hid_set_report_buf[1] == SET_REPORT_RESET) {
                                CyU3PDebugPrint(4, "Reset!\n");
                                break;
                            }
                        }
                    }
                }
                break;

            case REQ_SET_IDLE:
                CyU3PUsbAckSetup();
                if ((wValue & 0xff) == REPORT_ID_2) {
                    hid_idle_rate_2 = (unsigned char)(unsigned short)(wValue >> 8);
                } else if ((wValue & 0xff) == REPORT_ID_7) {
                    hid_idle_rate_7 = (unsigned char)(unsigned short)(wValue >> 8);
                } else {
                }
                break;

            case REQ_SET_PROTOCOL:
                CyU3PUsbAckSetup();
                break;

            default:
                break;
            }
        }
        break;

    case RQT_CLASS_INTERFACE:
        CyU3PDebugPrint(4, "Class Request Target Interface\r\n");
        break;

    case RQT_CLASS_ENDPOINT:
        CyU3PDebugPrint(4, "Class Request Target Endpoint\r\n");
        break;

    default:
        CyU3PDebugPrint(4, "Vendor Request\r\n");
        break;
    }

    return isHandled;
}

#define USB_EP_EVT_ALL   0xFFF    /* every CyU3PUsbEpEvtType */
#define USB_EP_1_TO_3    0x0E     /* endpoint bit mask, OUT and IN */

/* PIB clock, USB stack, callbacks and descriptors. The device is not connected here
 * (CyU3PConnectState(CyFalse, ...)); com_thread() connects once the cameras are configured. */
void usb_init(void)
{
    CyU3PReturnStatus_t status;
    CyU3PPibClock_t pibClock;

    status = CY_U3P_SUCCESS;
    pibClock.clkDiv = 2;
    pibClock.clkSrc = CY_U3P_SYS_CLK;
    pibClock.isDllEnable = CyFalse;
    pibClock.isHalfDiv = CyFalse;

    status = CyU3PPibInit(CyTrue, &pibClock);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "PIB Function Failed to Start, Error Code = %d\n", status);
    }

    status = CyU3PUsbStart();
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PUsbStart failed to Start, Error code = %d\n", status);
    }

    CyU3PUsbRegisterSetupCallback(usb_setup_cb, CyTrue);
    CyU3PUsbRegisterEventCallback(usb_event_cb);
    CyU3PUsbRegisterLPMRequestCallback(usb_lpm_request_cb);
    CyU3PUsbRegisterEpEvtCallback(usb_ep_event_cb, USB_EP_EVT_ALL, USB_EP_1_TO_3, USB_EP_1_TO_3);

    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set device descriptor failed, Error code = %d\n", status);
    }
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set device descriptor failed, Error code = %d\n", status);
    }
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set configuration descriptor failed, Error code = %d\n", status);
    }
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set device qualifier descriptor failed, Error code = %d\n", status);
    }
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set configuration descriptor failed, Error code = %d\n", status);
    }
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB Set Other Speed Descriptor failed, Error Code = %d\n", status);
    }
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB Set Configuration Descriptor failed, Error Code = %d\n", status);
    }
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set string descriptor failed, Error code = %d\n", status);
    }
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set string descriptor failed, Error code = %d\n", status);
    }
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set string descriptor failed, Error code = %d\n", status);
    }

    status = CyU3PConnectState(CyFalse, CyTrue);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB Connect failed, Error code = %d\n", status);
    }
}

/* I2C master at 100 kHz, register mode. */
void i2c_init(void)
{
    CyU3PReturnStatus_t status;
    CyU3PI2cConfig_t i2cConfig;

    status = CyU3PI2cInit();
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "I2C initialization failed!\n");
    }

    i2cConfig.bitRate = 100000;
    i2cConfig.isDma = CyFalse;
    i2cConfig.busTimeout = 0xFFFFFFFF;
    i2cConfig.dmaTimeout = 0xFFFF;

    status = CyU3PI2cSetConfig(&i2cConfig, NULL);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "I2C configuration failed!\n");
    }
}

/* IO matrix: 16-bit GPIF, UART + I2C + SPI, simple GPIOs 36, 37, 51, 52 (board.h). */

CyU3PReturnStatus_t io_matrix_init(void)
{
    CyU3PReturnStatus_t status;
    CyU3PIoMatrixConfig_t io_cfg;

    status = CY_U3P_SUCCESS;

    CyU3PMemSet((uint8_t *)&io_cfg, 0, sizeof(io_cfg));   /* s0Mode/s1Mode: leave the S-ports off */
    io_cfg.useUart = CyTrue;
    io_cfg.useI2C = CyTrue;
    io_cfg.useI2S = CyFalse;
    io_cfg.useSpi = CyTrue;
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    io_cfg.gpioSimpleEn[0] = 0;
    io_cfg.gpioSimpleEn[1] = GPIO_SIMPLE_EN_HI;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;

    status = CyU3PDeviceConfigureIOMatrix(&io_cfg);
    return status;
}

/* Same IO matrix again, then GPIO and SPI re-initialised (command 0xcc00, before reprogramming). */
CyU3PReturnStatus_t io_reconfigure(void)
{
    CyU3PReturnStatus_t status;
    CyU3PIoMatrixConfig_t io_cfg;

    status = CY_U3P_SUCCESS;
    status = CyU3PGpioDeInit();

    CyU3PMemSet((uint8_t *)&io_cfg, 0, sizeof(io_cfg));   /* s0Mode/s1Mode: leave the S-ports off */
    io_cfg.useUart = CyTrue;
    io_cfg.useI2C = CyTrue;
    io_cfg.useI2S = CyFalse;
    io_cfg.useSpi = CyTrue;
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    io_cfg.gpioSimpleEn[0] = 0;
    io_cfg.gpioSimpleEn[1] = GPIO_SIMPLE_EN_HI;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;

    status = CyU3PDeviceConfigureIOMatrix(&io_cfg);
    status = gpio_init();
    status = spi_init();

    return status;
}
