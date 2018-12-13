/**************************************************************************/
/*!
    @file     Adafruit_PN532.h
    @author   Adafruit Industries
	@license  BSD (see license.txt)


	This is a library for the Adafruit PN532 NFC/RFID breakout boards
	This library works with the Adafruit NFC breakout
	----> https://www.adafruit.com/products/364

	Check out the links above for our tutorials and wiring diagrams
  These chips use SPI or I2C to communicate.

	Adafruit invests time and resources providing this open source code,
	please support Adafruit and open-source hardware by purchasing
	products from Adafruit!

	@section  HISTORY

  v2.0  - Refactored to add I2C support from Adafruit_NFCShield_I2C library.

	v1.1  - Added full command list
          - Added 'verbose' mode flag to constructor to toggle debug output
          - Changed readPassiveTargetID() to return variable length values

*/
/**************************************************************************/

#define PN532_PREAMBLE                      (0x00)
#define PN532_STARTCODE1                    (0x00)
#define PN532_STARTCODE2                    (0xFF)
#define PN532_POSTAMBLE                     (0x00)

#define PN532_HOSTTOPN532                   (0xD4)
#define PN532_PN532TOHOST                   (0xD5)

// PN532 Commands
#define PN532_COMMAND_DIAGNOSE              (0x00)
#define PN532_COMMAND_GETFIRMWAREVERSION    (0x02)
#define PN532_COMMAND_GETGENERALSTATUS      (0x04)
#define PN532_COMMAND_READREGISTER          (0x06)
#define PN532_COMMAND_WRITEREGISTER         (0x08)
#define PN532_COMMAND_READGPIO              (0x0C)
#define PN532_COMMAND_WRITEGPIO             (0x0E)
#define PN532_COMMAND_SETSERIALBAUDRATE     (0x10)
#define PN532_COMMAND_SETPARAMETERS         (0x12)
#define PN532_COMMAND_SAMCONFIGURATION      (0x14)
#define PN532_COMMAND_POWERDOWN             (0x16)
#define PN532_COMMAND_RFCONFIGURATION       (0x32)
#define PN532_COMMAND_RFREGULATIONTEST      (0x58)
#define PN532_COMMAND_INJUMPFORDEP          (0x56)
#define PN532_COMMAND_INJUMPFORPSL          (0x46)
#define PN532_COMMAND_INLISTPASSIVETARGET   (0x4A)
#define PN532_COMMAND_INATR                 (0x50)
#define PN532_COMMAND_INPSL                 (0x4E)
#define PN532_COMMAND_INDATAEXCHANGE        (0x40)
#define PN532_COMMAND_INCOMMUNICATETHRU     (0x42)
#define PN532_COMMAND_INDESELECT            (0x44)
#define PN532_COMMAND_INRELEASE             (0x52)
#define PN532_COMMAND_INSELECT              (0x54)
#define PN532_COMMAND_INAUTOPOLL            (0x60)
#define PN532_COMMAND_TGINITASTARGET        (0x8C)
#define PN532_COMMAND_TGSETGENERALBYTES     (0x92)
#define PN532_COMMAND_TGGETDATA             (0x86)
#define PN532_COMMAND_TGSETDATA             (0x8E)
#define PN532_COMMAND_TGSETMETADATA         (0x94)
#define PN532_COMMAND_TGGETINITIATORCOMMAND (0x88)
#define PN532_COMMAND_TGRESPONSETOINITIATOR (0x90)
#define PN532_COMMAND_TGGETTARGETSTATUS     (0x8A)

#define PN532_RESPONSE_INDATAEXCHANGE       (0x41)
#define PN532_RESPONSE_INLISTPASSIVETARGET  (0x4B)

#define PN532_WAKEUP                        (0x55)

#define PN532_SPI_STATREAD                  (0x02)
#define PN532_SPI_DATAWRITE                 (0x01)
#define PN532_SPI_DATAREAD                  (0x03)
#define PN532_SPI_READY                     (0x01)

#define PN532_I2C_ADDRESS                   (0x48 >> 1)
#define PN532_I2C_READBIT                   (0x01)
#define PN532_I2C_BUSY                      (0x00)
#define PN532_I2C_READY                     (0x01)
#define PN532_I2C_READYTIMEOUT              (20)

#define PN532_MIFARE_ISO14443A              (0x00)

// Mifare Commands
#define MIFARE_CMD_AUTH_A                   (0x60)
#define MIFARE_CMD_AUTH_B                   (0x61)
#define MIFARE_CMD_READ                     (0x30)
#define MIFARE_CMD_WRITE                    (0xA0)
#define MIFARE_CMD_TRANSFER                 (0xB0)
#define MIFARE_CMD_DECREMENT                (0xC0)
#define MIFARE_CMD_INCREMENT                (0xC1)
#define MIFARE_CMD_STORE                    (0xC2)
#define MIFARE_ULTRALIGHT_CMD_WRITE         (0xA2)



  void begin(void);

  // Generic PN532 functions
  bool     SAMConfig(void);
  uint32_t getFirmwareVersion(void);
  bool     sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout);
  bool     writeGPIO(uint8_t pinstate);
  uint8_t  readGPIO(void);
  bool     setPassiveActivationRetries(uint8_t maxRetries);

  // ISO14443A functions
  bool readPassiveTargetID(uint8_t cardbaudrate, uint8_t * uid, uint8_t * uidLength, uint16_t timeout ); //timeout 0 means no timeout - will block forever.
  bool inDataExchange(uint8_t * send, uint8_t sendLength, uint8_t * response, uint8_t * responseLength);
  bool inListPassiveTarget();

  // Mifare Classic functions
  bool    mifareclassic_IsFirstBlock (uint32_t uiBlock);
  bool    mifareclassic_IsTrailerBlock (uint32_t uiBlock);
  uint8_t mifareclassic_AuthenticateBlock (uint8_t * uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t * keyData);
  uint8_t mifareclassic_ReadDataBlock (uint8_t blockNumber, uint8_t * data);
  uint8_t mifareclassic_WriteDataBlock (uint8_t blockNumber, uint8_t * data);
  uint8_t mifareclassic_FormatNDEF (void);
  uint8_t mifareclassic_WriteNDEFURI (uint8_t sectorNumber, uint8_t uriIdentifier, const char * url);


  //uint8_t _ss, _clk, _mosi, _miso;
enum
{
	_mosi		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	_clk		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	_ss		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	_miso		= GPIO_MAKE_PIN(HW_GPIOA, 8),
};
  uint8_t _irq, _reset;
  uint8_t _uid[7];       // ISO14443A uid
  uint8_t _uidLen;       // uid len
  uint8_t _key[6];       // Mifare Classic key
  uint8_t _inListedTag;  // Tg number of inlisted tag.
  bool    _usingSPI;     // True if using SPI, false if using I2C.
  bool    _hardwareSPI;  // True is using hardware SPI, false if using software SPI.

  // Low level communication functions that handle both SPI and I2C.
  void readdata(uint8_t* buff, uint8_t n);
  void writecommand(uint8_t* cmd, uint8_t cmdlen);
  bool isready();
  bool waitready(uint16_t timeout);
  bool readack();

  // SPI-specific functions.
  void    spi_write(uint8_t c);
  uint8_t spi_read(void);

  // Note there are i2c_read and i2c_write inline functions defined in the .cpp file.
