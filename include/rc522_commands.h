#pragma once

// MFRC522 commands
typedef enum rc522_cmd {
    RC522_CMD_IDLE 			= 0x00,
    RC522_CMD_MEM 			= 0x01,
    RC522_CMD_GEN_RANDOM_ID = 0x02,
    RC522_CMD_CALC_CRC 		= 0x03,
    RC522_CMD_TRANSMIT      = 0x04,
    RC522_CMD_NO_CMD_CHANGE = 0x07,
    RC522_CMD_RECEIVE       = 0x08,
    RC522_CMD_TRANSCEIVE    = 0x0C,
    RC522_CMD_AUTHENT       = 0x0E,
    RC522_CMD_SOFTRESET     = 0x0F,
} rc522_cmd_t;