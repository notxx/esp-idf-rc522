#pragma once

/**
 * @brief Constants defined in the ISO_IEC_14443 Protocol.
 *  This constants will be put in the FIFO to be read by the card itself.
 *  A handshake following the protocol have to be done before exchange of data. 
 */ 

#define RC522_REQA                0x26
#define RC522_WUPA                0x52
#define RC522_ANTICOLLISION_CMD   0x93
#define RC522_SELECT_CMD          0x93
#define RC522_WRITE_CMD           0xA0