/* spi_flash.c -- SPI flash access (after the SDK's SPI example) and FPGA configuration. */
#include "app.h"

/* SPI flash commands */
#define FLASH_CMD_WREN        0x06
#define FLASH_CMD_VSR_WREN    0x50       /* volatile status-register write enable */
#define FLASH_CMD_RDSR1       0x05
#define FLASH_CMD_RDSR2       0x35
#define FLASH_CMD_RDSR3       0x15
#define FLASH_CMD_READ        0x03
#define FLASH_CMD_PP          0x02       /* page program */
#define FLASH_CMD_SE_4K       0x20
#define FLASH_CMD_BE_64K      0xD8
#define FLASH_SR_BUSY         0x01
#define FLASH_SR_WEL          0x02

#define FLASH_PAGE_SHIFT      8
#define FLASH_BLOCK_SHIFT     16         /* 64 KB erase block */
#define FLASH_BLOCK_ERASE_MS  150        /* fixed wait, no status poll */
#define SPI_CLOCK_HZ          30000000

/* FPGA configuration */
#define FPGA_BITSTREAM_PAGES  384        /* 384 x 4 KB = 1.5 MB */

uint8_t flash_page_buf[FLASH_SECTOR_SIZE] = {0};      /* scratch: fpga_load() reads the bitstream through it */

/* Chip select. Only device 0 (the SPI SSN line) exists; device 1 is a stub. */
CyU3PReturnStatus_t flash_select(uint8_t dev, CyBool_t high)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    switch (dev) {
    case 0:
        status = CyU3PSpiSetSsnLine(high);
        break;
    case 1:
        ;
        break;
    default:
        break;
    }

    return status;
}

/* SPI master: mode 3, 30 MHz, 8-bit words, firmware-controlled SSN. */
CyU3PReturnStatus_t spi_init(void)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PSpiConfig_t spiConfig;

    status = CyU3PSpiInit();
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PSpiInit failed, Error code = %d\n", status);
        return status;
    }

    CyU3PMemSet((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyFalse;
    spiConfig.cpol       = CyTrue;
    spiConfig.ssnPol     = CyFalse;
    spiConfig.cpha       = CyTrue;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
    spiConfig.clock      = SPI_CLOCK_HZ;
    spiConfig.wordLen    = 8;

    status = CyU3PSpiSetConfig(&spiConfig, NULL);
    if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PSpiSetConfig failed, Error code = %d\n", status);
        return status;
    }
    return status;
}

/* WREN + RDSR until the flash is idle and write-enabled (the SDK's CyFxSpiWaitForStatus). */
CyU3PReturnStatus_t flash_wait_ready(uint8_t dev)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t cmd[4];
    uint8_t sr[4];

    do {
        cmd[0] = FLASH_CMD_WREN;
        flash_select(dev, CyFalse);
        status = CyU3PSpiTransmitWords(cmd, 1);
        flash_select(dev, CyTrue);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(2, "SPI WR_ENABLE command failed\n\r");
            return status;
        }

        cmd[0] = FLASH_CMD_RDSR1;
        flash_select(dev, CyFalse);
        status = CyU3PSpiTransmitWords(cmd, 1);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(2, "SPI READ_STATUS command failed\n\r");
            flash_select(dev, CyTrue);
            return status;
        }

        status = CyU3PSpiReceiveWords(sr, 2);
        flash_select(dev, CyTrue);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(2, "SPI status read failed\n\r");
            return status;
        }
    } while ((sr[0] & FLASH_SR_BUSY) != 0 || (sr[0] & FLASH_SR_WEL) == 0);

    return CY_U3P_SUCCESS;
}

/* Erase one 4 KB sector (sector number, not address). */
CyU3PReturnStatus_t flash_erase_sector(uint16_t sector, uint8_t dev)
{
    uint32_t addr;
    CyU3PReturnStatus_t status;
    uint8_t cmd[4];

    addr = 0;
    status = CY_U3P_SUCCESS;
    cmd[0] = FLASH_CMD_WREN;

    status = flash_wait_ready(dev);
    flash_select(dev, CyFalse);
    status = CyU3PSpiTransmitWords(cmd, 1);
    flash_select(dev, CyTrue);
    if (status != CY_U3P_SUCCESS)
        return status;

    cmd[0] = FLASH_CMD_SE_4K;
    addr = sector;
    addr = addr << FLASH_SECTOR_SHIFT;
    cmd[1] = (addr >> 16) & 0xff;
    cmd[2] = (addr >> 8) & 0xff;
    cmd[3] = addr & 0xff;
    flash_select(dev, CyFalse);
    status = CyU3PSpiTransmitWords(cmd, 4);
    flash_select(dev, CyTrue);
    status = flash_wait_ready(dev);
    return status;
}

/* Erase one 64 KB block (block number), then wait a fixed 150 ms. */
CyU3PReturnStatus_t flash_erase_block(uint8_t block, uint8_t dev)
{
    uint32_t addr = 0;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t cmd[4];

    cmd[0] = FLASH_CMD_WREN;
    flash_select(dev, CyFalse);
    status = CyU3PSpiTransmitWords(cmd, 1);
    flash_select(dev, CyTrue);
    if (status != CY_U3P_SUCCESS)
        return status;

    cmd[0] = FLASH_CMD_BE_64K;
    addr = block;
    addr = addr << FLASH_BLOCK_SHIFT;
    cmd[1] = (addr >> 16) & 0xff;
    cmd[2] = (addr >> 8) & 0xff;
    cmd[3] = addr & 0xff;
    flash_select(dev, CyFalse);
    status = CyU3PSpiTransmitWords(cmd, 4);
    flash_select(dev, CyTrue);
    CyU3PThreadSleep(FLASH_BLOCK_ERASE_MS);
    return status;
}

/* Erase the 64 KB blocks covering [0, size) -- always from address 0. Command 0xcc00 uses it with
 * size 0x1000, i.e. erases 0x000000-0x00FFFF. */
int flash_erase_range(uint8_t dev, uint32_t size)
{
    uint32_t i;
    uint32_t blocks;
    int result = 0;

    blocks = size >> FLASH_BLOCK_SHIFT;
    if ((uint16_t)size != 0)
    {
        blocks = blocks + 1;
    }

    for (i = 0; i < blocks; i++)
    {
        flash_erase_block((uint8_t)i, dev);
    }

    return result;
}

/* Read or write `size` bytes at 256-byte page `page` (the SDK's CyFxSpiTransfer). */
CyU3PReturnStatus_t flash_rw_pages(uint16_t page, uint16_t size, uint8_t *buffer,
                                   uint8_t dev, CyBool_t isRead)
{
    uint32_t addr = 0;
    uint16_t pageCount = size >> FLASH_PAGE_SHIFT;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t cmd[4];

    if (size == 0)
        return CY_U3P_SUCCESS;

    if (size & 0xff)
        pageCount++;

    addr = page;
    addr = addr << FLASH_PAGE_SHIFT;
    CyU3PDebugPrint(2, "SPI access - addr: 0x%x, size: 0x%x, pages: 0x%x.\r\n", addr, size, pageCount);

    while (pageCount != 0) {
        cmd[1] = (addr >> 16) & 0xff;
        cmd[2] = (addr >> 8) & 0xff;
        cmd[3] = addr & 0xff;

        if (isRead != 0) {
            cmd[0] = FLASH_CMD_READ;
            status = flash_wait_ready(dev);
            if (status != CY_U3P_SUCCESS)
                return status;
            flash_select(dev, CyFalse);
            status = CyU3PSpiTransmitWords(cmd, 4);
            if (status != CY_U3P_SUCCESS) {
                CyU3PDebugPrint(2, "SPI READ command failed\r\n");
                flash_select(dev, CyTrue);
                return status;
            }
            status = CyU3PSpiReceiveWords(buffer, FLASH_PAGE_SIZE);
            if (status != CY_U3P_SUCCESS) {
                flash_select(dev, CyTrue);
                return status;
            }
            flash_select(dev, CyTrue);
        } else {
            cmd[0] = FLASH_CMD_PP;
            status = flash_wait_ready(dev);
            if (status != CY_U3P_SUCCESS)
                return status;
            flash_select(dev, CyFalse);
            status = CyU3PSpiTransmitWords(cmd, 4);
            if (status != CY_U3P_SUCCESS) {
                CyU3PDebugPrint(2, "SPI WRITE command failed\r\n");
                flash_select(dev, CyTrue);
                return status;
            }
            status = CyU3PSpiTransmitWords(buffer, FLASH_PAGE_SIZE);
            if (status != CY_U3P_SUCCESS) {
                flash_select(dev, CyTrue);
                return status;
            }
            flash_select(dev, CyTrue);
        }

        addr += FLASH_PAGE_SIZE;
        buffer += FLASH_PAGE_SIZE;
        pageCount--;
    }
    return CY_U3P_SUCCESS;
}

/* Configure the FPGA: pulse PROGRAM, then read the 1.5 MB bitstream of the active slot
 * (fpga_load_addr_get()) from flash while the FPGA listens on the SPI bus (GPIO 51 low). */
CyU3PReturnStatus_t fpga_load(void)
{
    int page;
    CyU3PReturnStatus_t status;
    uint32_t addr;
    union { uint8_t b[4]; uint32_t w; } cmd;

    page = 0;
    status = CY_U3P_SUCCESS;
    addr = 0;
    cmd.w = 0;

    CyU3PGpioSimpleSetValue(GPIO_FPGA_PROGRAM, CyFalse);
    CyU3PThreadSleep(10);
    CyU3PGpioSimpleSetValue(GPIO_FPGA_PROGRAM, CyTrue);
    CyU3PThreadSleep(10);

    addr = fpga_load_addr_get();
    for (page = 0; page < FPGA_BITSTREAM_PAGES; page++) {
        cmd.b[0] = FLASH_CMD_READ;
        cmd.b[1] = (addr >> 16) & 0xff;
        cmd.b[2] = (addr >> 8) & 0xff;
        cmd.b[3] = addr & 0xff;

        status = flash_wait_ready(0);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(2, "Spi wait status failed, address: 0x%x failed\r\n", addr);
            return status;
        }

        flash_select(0, CyFalse);
        status = CyU3PSpiTransmitWords(cmd.b, 4);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(2, "Spi read command failed, address: 0x%x failed\r\n", addr);
            return status;
        }

        CyU3PGpioSimpleSetValue(GPIO_FPGA_CFG_SEL, CyFalse);
        status = CyU3PSpiReceiveWords(flash_page_buf, FLASH_SECTOR_SIZE);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(2, "Spi receive command failed, address: 0x%x failed\r\n", addr);
            return status;
        }
        CyU3PGpioSimpleSetValue(GPIO_FPGA_CFG_SEL, CyTrue);
        flash_select(0, CyTrue);

        addr = addr + FLASH_SECTOR_SIZE;
    }
    return status;
}

/* Write a status register: `cmd` = WRSR1/2/3 (0x01/0x31/0x11), preceded by WREN, or by the
 * volatile-SR enable (0x50) when `isVolatile`. */
int flash_write_status_reg(uint8_t cmd, uint8_t value, uint8_t isVolatile)
{
    union { uint32_t w; uint8_t b[4]; } buf;

    buf.w = 0;
    if (isVolatile == 0)
        buf.b[0] = FLASH_CMD_WREN;
    else
        buf.b[0] = FLASH_CMD_VSR_WREN;

    CyU3PSpiSetSsnLine(CyFalse);
    CyU3PSpiTransmitWords(buf.b, 1);
    CyU3PSpiSetSsnLine(CyTrue);

    CyU3PSpiSetSsnLine(CyFalse);
    buf.b[0] = cmd;
    buf.b[1] = value;
    CyU3PSpiTransmitWords(buf.b, 2);
    CyU3PSpiSetSsnLine(CyTrue);

    CyU3PThreadSleep(10);
    return 0;
}

/* Read status registers 1-3 into sr[0..2]. */
int flash_read_status_regs(uint8_t *sr)
{
    union { int w; uint8_t b; } buf;

    buf.w = 0;
    if (sr == NULL)
        return -1;

    CyU3PSpiSetSsnLine(CyFalse);
    buf.b = FLASH_CMD_RDSR1;
    CyU3PSpiTransmitWords(&buf.b, 1);
    CyU3PSpiReceiveWords(&buf.b, 1);
    CyU3PSpiSetSsnLine(CyTrue);
    sr[0] = buf.b;

    CyU3PSpiSetSsnLine(CyFalse);
    buf.b = FLASH_CMD_RDSR2;
    CyU3PSpiTransmitWords(&buf.b, 1);
    CyU3PSpiReceiveWords(&buf.b, 1);
    CyU3PSpiSetSsnLine(CyTrue);
    sr[1] = buf.b;

    CyU3PSpiSetSsnLine(CyFalse);
    buf.b = FLASH_CMD_RDSR3;
    CyU3PSpiTransmitWords(&buf.b, 1);
    CyU3PSpiReceiveWords(&buf.b, 1);
    CyU3PSpiSetSsnLine(CyTrue);
    sr[2] = buf.b;

    return 0;
}

/* GPIO interrupt callback (registered by gpio_init()). The SDK example's structure with the
 * actions removed: it reads the pin and tests for GPIO 36, doing nothing either way. */
void gpio_interrupt_cb(uint8_t gpioId)
{
    CyU3PReturnStatus_t status;
    CyBool_t gpioValue = CyFalse;

    status = CY_U3P_SUCCESS;
    status = CyU3PGpioGetValue(gpioId, &gpioValue);
    if (status == CY_U3P_SUCCESS) {
        if (gpioId == 36) {
            if (gpioValue) {
            }
        }
    }
}
