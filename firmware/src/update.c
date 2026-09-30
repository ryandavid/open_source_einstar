/* update.c -- the bulk channel (EP 0x02/0x82): firmware update into the inactive A/B slot, the user
 * flash pages, and the boot record that selects the slot. */
#include "app.h"

#define BULK_REPLY_SIZE     1024          /* bytes sent back for a bulk reply */
#define BULK_REPLY_SIZE_PAGE 0x1400       /* 10/57: 9-byte header + 4 KB page, rounded up */
#define UPDATE_PAGE_SIZE    FLASH_SECTOR_SIZE

/* 00/06 first packet: payload byte 0 (not used), then the firmware size (BE32, payload bytes
 * 1..4); starts the update, and the following bulk packets go to update_write_page(). Returns
 * the reply size. */
int update_begin(pkt_req_t *req, pkt_reply_t *reply)
{
    int result;
    unsigned int size;

    size = 0;
    result = BULK_REPLY_SIZE;

    reply->length_be = 0;

    CyU3PMemCopy((uint8_t *)&size, &req->arg0 + 1, 4);

    size = BSWAP32(size);

    app_state.update_size = size;
    app_state.update_active = 1;

    CyU3PTimerStart(&update_timer);

    CyU3PDebugPrint(4, "Firmware size is %d, update starting...\r\n", app_state.update_size);

    return result;
}

/* One 4 KB update page: payload byte 0 (not used), the data, then one check byte (8-bit sum of
 * the data); the payload length is the data length + 2. On the first page the target slot is
 * chosen from the boot record: the slot that does not boot. Each page is erased, written, read
 * back and summed. After the announced size the boot record is rewritten to boot the new slot and
 * the device reboots (emc_wdg_thread). Any error abandons the update. */
typedef struct __attribute__((packed)) {    /* boot record (flash page BOOT_RECORD_PAGE) */
    uint32_t new_app;                       /* uiNewAppAddr: the slot written last */
    uint32_t old_app;                       /* uiOldAppAddr */
    uint8_t  boot_new;                      /* ucIsBootNewApp: 1 = boot new_app */
    uint8_t  rest[251];                     /* receives the rest of the flash page read; the
                                             * vendor's object is 260 bytes */
} boot_record_t;
#define BOOT_RECORD_WRITE_SIZE  12          /* bytes written back: sizeof of the unpacked
                                             * {u32, u32, u8} record */

uint8_t page_buf[UPDATE_PAGE_SIZE + 1] = {0};   /* one update page read back (+1: unused) */
boot_record_t boot_record = {0};
int update_page_index = 0;                  /* pages written in this update */
int update_bytes_received = 0;

int update_write_page(pkt_req_t *req, pkt_reply_t *reply)
{
    int i;
    unsigned char sum2;
    int result;
    unsigned int reqLen;
    unsigned char *data;
    unsigned char sum1;
    boot_record_t *rec;

    result = BULK_REPLY_SIZE;
    reqLen = 0;
    i = 0;
    data = 0;
    sum1 = 0;
    sum2 = 0;
    rec = 0;

    reqLen = BSWAP32(req->length_be) - 2;

    if (reqLen > UPDATE_PAGE_SIZE) {
        CyU3PDebugPrint(4, "Firmware update package size is 0x%x, TOO BIG\r\n", reqLen);
        goto cleanup_error;
    }

    data = &req->arg0 + 1;
    sum1 = data[reqLen];

    flash_lock();
    CyU3PTimerStop(&update_timer);
    CyU3PTimerModify(&update_timer, UPDATE_TIMEOUT_MS, UPDATE_TIMEOUT_MS);
    CyU3PTimerStart(&update_timer);
    rec = &boot_record;

    if (update_page_index == 0) {
        flash_rw_pages(BOOT_RECORD_PAGE, FLASH_PAGE_SIZE, (uint8_t *)&boot_record, 0, 1);

        CyU3PDebugPrint(4, "Before pstAppFlag->uiNewAppAddr: 0x%x, pstAppFlag->uiOldAppAddr: 0x%x\r\n", rec->new_app, rec->old_app);

        if (rec->new_app == SLOT_B_APP) {
            if (rec->boot_new == 1) {
                rec->new_app = SLOT_A_APP;
                rec->old_app = SLOT_B_APP;
            } else {
                rec->new_app = SLOT_B_APP;
                rec->old_app = SLOT_A_APP;
            }
        } else if (rec->new_app == SLOT_A_APP) {
            if (rec->boot_new == 1) {
                rec->new_app = SLOT_B_APP;
                rec->old_app = SLOT_A_APP;
            } else {
                rec->new_app = SLOT_A_APP;
                rec->old_app = SLOT_B_APP;
            }
        } else {
            rec->new_app = SLOT_B_APP;
            rec->old_app = SLOT_A_APP;
        }

        CyU3PDebugPrint(4, "After pstAppFlag->uiNewAppAddr: 0x%x, pstAppFlag->uiOldAppAddr: 0x%x\r\n", rec->new_app, rec->old_app);
        rec->boot_new = 1;
    }

    flash_erase_sector((unsigned short)(rec->new_app >> FLASH_SECTOR_SHIFT) + (unsigned short)update_page_index, 0);
    flash_rw_pages((unsigned short)(rec->new_app / FLASH_PAGE_SIZE) +
                     (unsigned short)((unsigned short)update_page_index << (FLASH_SECTOR_SHIFT - 8)),
                 UPDATE_PAGE_SIZE, data, 0, 0);
    flash_rw_pages((unsigned short)(rec->new_app / FLASH_PAGE_SIZE) +
                     (unsigned short)((unsigned short)update_page_index << (FLASH_SECTOR_SHIFT - 8)),
                 UPDATE_PAGE_SIZE, page_buf, 0, 1);
    flash_unlock();

    for (i = 0; i < reqLen; i++) {
        sum2 = sum2 + page_buf[i];
    }

    if (sum1 != sum2) {
        CyU3PDebugPrint(4, "Firmware update package ucCheckByte(0x%x) != ucCheckCal(0x%x)\r\n", sum1, sum2);
        goto cleanup_error;
    }

    update_page_index = update_page_index + 1;
    update_bytes_received = update_bytes_received + (int)reqLen;
    reply->status = ST_OK;
    if (app_state.update_size <= (unsigned int)update_bytes_received) {
        CyU3PDebugPrint(4, "Firmware update over, firmware size %d, recive bytes %d\r\n", app_state.update_size, update_bytes_received);
        flash_lock();
        flash_erase_sector(BOOT_RECORD_SECTOR, 0);
        flash_rw_pages(BOOT_RECORD_PAGE, BOOT_RECORD_WRITE_SIZE, (uint8_t *)&boot_record, 0, 0);
        flash_unlock();
        update_page_index = 0;
        update_bytes_received = 0;
        app_state.update_active = 0;
        app_state.update_done = 1;
    }
    return result;

cleanup_error:
    update_page_index = 0;
    update_bytes_received = 0;
    app_state.update_active = 0;
    reply->status = ST_BAD_LENGTH;
    CyU3PDebugPrint(4, "Firmware update error\r\n");

    return result;
}

/* 10/57: read user page n (0..255, BE16 payload) = flash 0x400000 + n * 4 KB into the reply data;
 * reply length 4096. */
int user_page_read(pkt_req_t *req, pkt_reply_t *reply)
{
    int status = BULK_REPLY_SIZE_PAGE;
    unsigned short page = 0;

    reply->length_be = BSWAP32((uint32_t)FLASH_SECTOR_SIZE);

    CyU3PMemCopy((uint8_t *)&page, req->payload, 2);

    page = (page >> 8) | (page << 8);

    if (page > USER_AREA_PAGES - 1) {
        reply->status = ST_BAD_LENGTH;
        return -1;
    }

    flash_lock();
    flash_rw_pages((((int)page + USER_AREA_SECTOR) << FLASH_SECTOR_SHIFT) / FLASH_PAGE_SIZE,
                   FLASH_SECTOR_SIZE, reply->data, 0, 1);
    flash_unlock();

    return status;
}

/* 10/58: erase and write user page n (BE16 at payload bytes 0..1), 4096 bytes of data after it.
 * Only a zero payload length is rejected; the reply length is 4 (with no data). */
int user_page_write(pkt_req_t *req, pkt_reply_t *reply)
{
    int ret = BULK_REPLY_SIZE;
    unsigned short page = 0;

    reply->length_be = BSWAP32(4u);

    if (BSWAP32(req->length_be) == 0) {
        reply->status = ST_BAD_LENGTH;
        return -1;
    }

    CyU3PMemCopy((unsigned char *)&page, (unsigned char *)&req->page_be, 2);

    page = (page >> 8) | (page << 8);

    if (page > USER_AREA_PAGES - 1) {
        reply->status = ST_BAD_LENGTH;
        return -1;
    }

    flash_lock();
    flash_erase_sector((unsigned short)(page + USER_AREA_SECTOR), 0);

    flash_rw_pages((unsigned short)(((page + USER_AREA_SECTOR) << FLASH_SECTOR_SHIFT) / FLASH_PAGE_SIZE),
                   FLASH_SECTOR_SIZE, (uint8_t *)(&req->page_be + 1), 0, 0);

    flash_unlock();
    return ret;
}

/* The bulk channel. Requests have the command channel's header; the key is looked up in
 * bulk_handlers, whose non-zero sizes must match the BE32 payload length. While an update is
 * running every packet goes to update_write_page(). A repeated sequence number gets no reply
 * (no 0xFE/0xFF exception here). Returns the number of reply bytes to send. */
typedef int (*bulk_handler_t)(pkt_req_t *req, pkt_reply_t *reply);
struct bulk_cmd {
    unsigned short key;          /* group << 8 | opcode */
    unsigned int   length;       /* expected payload length, 0 = any */
    bulk_handler_t fn;           /* returns the reply size, or -1 on error */
};
struct bulk_cmd bulk_handlers[3] = {
    {CMD_FW_UPDATE,       5, update_begin},
    {CMD_USER_PAGE_READ,  2, user_page_read},
    {CMD_USER_PAGE_WRITE, 0, user_page_write},
};
uint16_t bulk_last_seq = SEQ_NONE;          /* sequence number of the last bulk command */

int bulk_dispatch(void *req, void *resp_buf)
{
    unsigned int i = 0;
    int result = BULK_REPLY_SIZE;
    pkt_req_t *p = 0;
    pkt_reply_t *resp = 0;

    p = req;
    if (p == 0)
        return -1;

    resp = (pkt_reply_t *)resp_buf;
    if (resp == 0)
        return -1;

    if (p->seq == bulk_last_seq) {
        bulk_last_seq = -1;
        return -1;
    }
    bulk_last_seq = p->seq;

    resp->seq = p->seq;
    resp->unmasked = PKT_REPLY_UNMASKED;
    resp->key_raw = p->key_raw;
    resp->length_be = 0;

    if (app_state.update_active != 0) {
        return update_write_page(p, resp);
    }

    for (i = 0; i <= 2; i++) {
        if (((p->group << 8) | p->opcode) != bulk_handlers[i].key) {
            resp->status = ST_UNKNOWN;
            continue;
        }

        if (bulk_handlers[i].length != 0) {
            if (BSWAP32(p->length_be) != bulk_handlers[i].length) {
                resp->status = ST_BAD_LENGTH;
                break;
            }
        }

        resp->status = ST_FAILED;
        if (bulk_handlers[i].fn != 0) {
            result = bulk_handlers[i].fn(p, resp);
            if (result == -1) {
                result = BULK_REPLY_SIZE;
                resp->length_be = 0;
            } else {
                resp->status = ST_OK;
            }
            break;
        }
    }

    return result;
}

/* Flash address of the FPGA bitstream to load: the slot the boot record boots. */
uint32_t fpga_load_addr_get(void)
{
    int addr = 0;
    boot_record_t *p = 0;
    unsigned char buf[FLASH_PAGE_SIZE] = {0};

    flash_rw_pages(BOOT_RECORD_PAGE, FLASH_PAGE_SIZE, buf, 0, 1);
    p = (boot_record_t *)buf;

    CyU3PDebugPrint(4, "uiNewAppAddr: 0x%x, uiOldAppAddr: 0x%x, ucIsBootNewApp: 0x%x\r\n", p->new_app, p->old_app, p->boot_new);

    if (p->new_app == SLOT_B_APP)
    {
        if (p->boot_new == 1)
        {
            addr = SLOT_B_FPGA;
        }
        else
        {
            addr = SLOT_A_FPGA;
        }
    }
    else if (p->new_app == SLOT_A_APP)
    {
        if (p->boot_new == 1)
        {
            addr = SLOT_A_FPGA;
        }
        else
        {
            addr = SLOT_B_FPGA;
        }
    }
    else
    {
        addr = SLOT_A_FPGA;
    }

    CyU3PDebugPrint(4, "GetFpgaLoadAddr: 0x%x\r\n", addr);

    return addr;
}
