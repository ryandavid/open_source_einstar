/* Command-channel keys: (group byte cmd[2] << 8) | opcode cmd[3], written "gg/oo" in
 * docs/protocol-device.md. Names follow that document; "(?)" marks keys it does not describe. */
#ifndef PROTOCOL_H
#define PROTOCOL_H

/* Packet layout, both channels (docs/protocol-device.md section 1). Requests: seq, mask key,
 * group, opcode, BE32 payload length, payload. Replies: seq, 2, group, opcode, status, BE32 data
 * length, data. The firmware never unmasks: it ignores byte 1 of a request and always sends
 * 2 ("not masked") in byte 1 of a reply. */
#define PKT_SEQ           0
#define PKT_MASK          1
#define PKT_GROUP         2
#define PKT_OPCODE        3
#define PKT_LENGTH        4       /* request: payload length */
#define PKT_PAYLOAD       8       /* request: payload */
#define PKT_STATUS        4       /* reply: status */
#define PKT_DATA_LENGTH   5       /* reply: data length */
#define PKT_DATA          9       /* reply: data */

#define PKT_REPLY_UNMASKED  2     /* reply byte 1 */

/* Reply status */
#define ST_OK             0
#define ST_UNKNOWN        1       /* bulk channel: unknown command */
#define ST_BAD_LENGTH     2       /* payload length (or, for 10/70 and updates, a value) rejected */
#define ST_FAILED         3       /* command channel: failed, or unknown command */
#define ST_5              5       /* reply_status(): unreachable */

/* Sequence numbers: the last one seen is kept to drop retransmissions. 0xFE and 0xFF are the
 * transport's own queries and are never treated as repeats (command channel only). */
#define SEQ_NONE          0xffff
#define SEQ_TRANSPORT_FE  0xfe
#define SEQ_TRANSPORT_FF  0xff

/* Request / reply as structures (bulk channel). Packed: gcc then reads the multi-byte fields
 * byte by byte, as the vendor's code does. The key is two aliases of the same halfword (see
 * bulk_dispatch()). */
typedef struct __attribute__((packed)) {
    uint8_t  seq;
    uint8_t  mask;
    union {
        uint16_t key_raw;         /* group, opcode as a little-endian halfword (echoed in the reply) */
        struct { uint8_t group, opcode; };
    };
    uint32_t length_be;           /* payload length, as stored (big-endian) */
    union {                       /* payload; named views of its start (they change the code:
                                   * `&arg0 + 1` is two additions, `payload + 1` one) */
        uint8_t  payload[0];
        uint8_t  arg0;            /* bulk 00/06: not used, then the data */
        uint16_t page_be;         /* bulk 10/57, 10/58: user page number, big-endian */
    };
} pkt_req_t;

typedef struct __attribute__((packed)) {
    uint8_t  seq;
    uint8_t  unmasked;            /* PKT_REPLY_UNMASKED */
    uint16_t key_raw;             /* echo of the request's group, opcode */
    uint8_t  status;
    uint32_t length_be;           /* data length, as stored (big-endian) */
    uint8_t  data[];
} pkt_reply_t;

#define BSWAP32(x)  (((x) >> 24) | (((x) & 0x00ff0000) >> 8) | (((x) & 0x0000ff00) << 8) | ((x) << 24))

#define CMD_VENDOR_NAME        0x0000   /* "Shining3D" */
#define CMD_PRODUCT_NAME       0x0001   /* "EinScan10_01" */
#define CMD_02                 0x0002   /* (?) replies 0 */
#define CMD_03                 0x0003   /* (?) replies 1 */
#define CMD_SERIAL             0x0004   /* FX3 eFuse die ID */
#define CMD_FW_VERSION         0x0005
#define CMD_DEVICE_STATE       0x0007   /* buttons; also the EP 0x83 recovery hook */
#define CMD_FW_UPDATE          0x0006   /* bulk: firmware update (IAP) */
#define CMD_REBOOT             0x0008
#define CMD_VENDOR_NAME_LEN    0x0009   /* (?) 18 */
#define CMD_PRODUCT_NAME_LEN   0x000A   /* (?) 12 */
#define CMD_FW_VERSION_LEN     0x000B   /* (?) 42 */
#define CMD_0C                 0x000C   /* (?) replies 7 */
#define CMD_0D                 0x000D   /* (?) FPGA reg 19 := 0, cancels reboot/update */
#define CMD_BB                 0x00BB   /* (?) FPGA reg 11 */
#define CMD_EF                 0x00EF   /* (?) FPGA reg 12 := 1 */
#define CMD_ERASE_APP_HEADER   0xCC00   /* erases flash 0x000000-0x00FFFF and reboots */
#define CMD_CCCC               0xCCCC   /* (?) raw FX3 GPIO 50 and 52 pin registers */
#define CMD_VID                0x1000
#define CMD_PID                0x1001
#define CMD_MAX_WIDTH          0x1016
#define CMD_MAX_HEIGHT         0x1017
#define CMD_MAX_EXPOSURE       0x1020
#define CMD_MIN_EXPOSURE       0x1021
#define CMD_GET_EXPOSURE       0x1022
#define CMD_SET_EXPOSURE       0x1023
#define CMD_MAX_GAIN           0x1024
#define CMD_MIN_GAIN           0x1025
#define CMD_GET_GAIN           0x1026
#define CMD_SET_GAIN           0x1027
#define CMD_PIXEL_BITS         0x102E   /* (?) replies 8 */
#define CMD_36                 0x1036   /* (?) no data */
#define CMD_37                 0x1037   /* (?) no effect */
#define CMD_GET_REG6_MODE      0x103E   /* (?) FPGA reg 6 */
#define CMD_SET_REG6_MODE      0x103F   /* (?) FPGA reg 6 */
#define CMD_TRIGGER_SWITCH_RD  0x1040
#define CMD_TRIGGER_SWITCH_WR  0x1041
#define CMD_TRIGGER_WORD_RD    0x1042   /* (?) 32-bit form of 10/40 */
#define CMD_TRIGGER_WORD_WR    0x1043   /* (?) 32-bit form of 10/41 */
#define CMD_TRIGGER_PERIOD_RD  0x1048
#define CMD_TRIGGER_PERIOD_WR  0x1049
#define CMD_LD_WORKMODE_RD     0x104E
#define CMD_LD_WORKMODE_WR     0x104F
#define CMD_USER_PAGE_READ     0x1057   /* bulk: flash page read */
#define CMD_USER_PAGE_WRITE    0x1058   /* bulk: flash page write */
#define CMD_TEMPERATURE        0x1050
#define CMD_SENSOR_COUNT       0x1051
#define CMD_56                 0x1056   /* (?) replies 0x00100000 */
#define CMD_COLOUR_MODE        0x105D   /* replies 0 only if payload byte 2 == 2, else leaves the byte unset */
#define CMD_INDICATION         0x1062
#define CMD_LD_BRIGHTNESS_RD   0x1067
#define CMD_LD_BRIGHTNESS_WR   0x1068
#define CMD_STROBE_RD          0x106F
#define CMD_STROBE_WR          0x1070
#define CMD_REG13_RD           0x1071   /* (?) FPGA reg 13 */
#define CMD_REG13_WR_A         0x1072   /* (?) */
#define CMD_REG13_WR_B         0x1073   /* (?) */
#define CMD_REG13_WR_C         0x1075   /* (?) */
#define CMD_77                 0x1077   /* (?) no effect */
#define CMD_78                 0x1078   /* (?) no effect */
#define CMD_7A                 0x107A   /* (?) no effect */
#define CMD_CLEAR_STATE        0x107B   /* FPGA reg 12 := 4 (button state) */

#endif
