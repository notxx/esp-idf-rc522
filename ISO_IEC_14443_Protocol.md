1. Power up and initialize MFRC522
   - Configure registers, set mode, etc.

2. Enable the antenna
   - Set the Tx and Rx registers to enable the antenna.

3. Anti-Collision Loop
   a. Send Request command
      - Command: REQA (0x26) or WUPA (0x52)
   b. Perform anticollision loop
      - Send Anti-Collision command (0x93)
      - Send Cascade Tag (0x20)
      - Retrieve UID of the selected card

```  
      In the context of ISO/IEC 14443A, specifically for anti-collision and selection operations, the 0x93 command is used as the Anti-Collision command, and the 0x20 byte is part of the Cascade Tag.

      Here's a brief explanation:

      Anti-Collision Command (0x93): The Anti-Collision command (0x93) is used to initiate the anti-collision process. When multiple cards are present in the reader's field, they will respond to this command, and the reader can then perform anti-collision to select one specific card.

      Cascade Tag (0x20): The Cascade Tag is used during anti-collision to indicate whether there are more UID bits to follow. The presence of the Cascade Tag helps the reader understand if it needs to perform additional anti-collision iterations to obtain the complete UID. It's part of the response from the card.

      The sequence of sending 0x93 and 0x20 is a typical part of the anti-collision process. The reader sends the Anti-Collision command (0x93), and the cards in the field respond with their UIDs. If the Cascade Tag (0x20) is present in the response, it indicates that there are more bits to be sent in the UID. 
```


4. Select Card
   - Send Select command (0x93) with UID

5. Authentication (if required)
   - For MIFARE cards, perform authentication for the relevant sector.

6. Write Data to the Card
   a. Prepare data for writing
   b. Send Write command (0xA0) with block address and data

7. Read Acknowledgment
   - Read the response from the card to ensure successful data writing.

8. Disable the Antenna (optional)

# Relevant documentation
- MFRC522 documentation: https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
- PICC MIFARE Classic 1K - Mainstream contactless smart card IC documentation: https://www.mouser.com/datasheet/2/302/MF1S503x-89574.pdf
- MIFARE Type Identification: https://www.nxp.com/docs/en/application-note/AN10833.pdf
- MIFARE ISO/IEC 14443 PICC selection documentation: https://www.nxp.com/docs/en/application-note/AN10834.pdf
   - Set the Tx and Rx registers to disable the antenna.
