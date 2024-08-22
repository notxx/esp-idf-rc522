#pragma once

/**
 * @brief Constants defined in the MIFARE/ISO_IEC_14443 Protocol.
 *  This constants will be put in the FIFO to be read by the card itself.
 *  A handshake following the protocol have to be done before exchange of data.
 *  Documentation under: https://www.mouser.com/datasheet/2/302/MF1S503x-89574.pdf
 *                       9.1 - MIFARE Classic command overview 
 */ 

#define MIFARE_REQA                0x26
#define MIFARE_WUPA                0x52
#define MIFARE_ANTICOLLISION_CL_1  {0x93, 0x20}
#define MIFARE_ANTICOLLISION_CL_2  {0x95, 0x20}
#define MIFARE_SELECT_CL_1         {0x93, 0x70}
#define MIFARE_HALT                {0x50, 0x00}
#define MIFARE_AUTH_KEY_A          0x60
#define MIFARE_AUTH_KEY_B          0x61
#define MIFARE_READ                0x30
#define MIFARE_WRITE               0xA0
#define MIFARE_DECREMENT           0xC0
#define MIFARE_INCREMENT           0xC1
#define MIFARE_RESTORE             0xC2
#define MIFARE_TRANSFER            0xB0