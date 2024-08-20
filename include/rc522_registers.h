#pragma once

typedef enum rc522_reg {
    RC522_REG_COMMAND =             0x01, // Starts and stops command execution
    // Communication Interrupt Enable Register.
    // Control bits to enable and disable the passing of interrupt requests
    RC522_REG_COMM_IRQ_EN =         0x02,
    // Diverted Interrupt Enable Register.
    // Control bits to enable and disable the passing of interrupt requests
    RC522_REG_DIV_IRQ_EN =          0x03,
    RC522_REG_COMM_IRQ =            0x04, // Communication Interrupt request bits
    RC522_REG_DIV_IRQ =             0x05, // Diverted Interrupt request bits
    // Error bits showing the error status of the last command  executed
    RC522_REG_ERROR =               0x06,
    RC522_REG_STATUS_1 =            0x07, // communication status bits
    // Contains status bits of the receiver, transmitter and data mode detector
    RC522_REG_STATUS_2 =            0x08,
    RC522_REG_FIFO_DATA =           0x09, // Input and output of 64 byte FIFO buffer
    RC522_REG_FIFO_LEVEL =          0x0A, // Number of bytes stored in the FIFO buffer
    RC522_REG_WATER_LEVEL =         0x0A, // level for FIFO underflow and overflow warning
    RC522_REG_CONTROL =             0x0C, // Miscellaneous control register
    RC522_REG_BIT_FRAMING =         0x0D, // Adjustments for bit-oriented frames
    // bit position of the first bit-collision detected on the RF interface
    RC522_REG_COLL =                0x0E,
    // Defines general modes for transmitting and receiving
    RC522_REG_MODE =                0x11,
    RC522_REG_TX_MODE =             0x12, // defines transmission data rate and framing
    RC522_REG_RX_MODE =             0x13, // defines reception data rate and framing
    // Controls the logical behavior of the antenna driver pins TX1 and TX2
    RC522_REG_TX_CONTROL =          0x14,
    // Controls the setting of the transmission modulation
    RC522_REG_TX_ASK =              0x15,
    // MSB (higher bits) values of the CRC calculation
    RC522_REG_CRC_RESULT_MSB =      0x21,
    // LSB (lower bits) values of the CRC calculation
    RC522_REG_CRC_RESULT_LSB =      0x22,
    // Sets the modulation width
    RC522_REG_MOD_WIDTH =           0x24,
    // Configures the receiver gain
    RC522_REG_RF_CFG =              0x26,
    // Defines the mode of the timer
    RC522_REG_TIMER_MODE =          0x2A,
    // Defines the timer prescaler settings
    RC522_REG_TIMER_PRESCALER =     0x2B,
    // MSB (higher bits) value of 16-bit timer reload value
    RC522_REG_TIMER_RELOAD_MSB =    0x2C,
    // LSB (lower bits) value of 16-bit timer reload value
    RC522_REG_TIMER_RELOAD_LSB =    0x2D,
    // Shows the MFRC522 software version
    RC522_REG_VERSION =             0x37,
} rc522_reg_t;