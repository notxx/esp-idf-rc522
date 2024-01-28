#pragma once



// MFRC522 commands
#define RC522_CMD_IDLE           0x00
#define RC522_CMD_MEM            0x01
#define RC522_CMD_GEN_RANDOM_ID  0x02
#define RC522_CMD_CALC_CRC       0x03
#define RC522_CMD_TRANSMIT       0x04
#define RC522_CMD_NO_CMD_CHANGE  0x07
#define RC522_CMD_RECEIVE        0x08
#define RC522_CMD_TRANSCEIVE     0x0C
#define RC522_CMD_AUTHENT        0x0E
#define RC522_CMD_SOFTRESET      0x0F