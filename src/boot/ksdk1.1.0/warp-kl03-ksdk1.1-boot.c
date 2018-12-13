/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"




#define					kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define					kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define					kWarpConstantStringErrorSanity		"\rSanity Check Failed!"

/*
 *	TODO: move this and possibly others into a global structure
 */
volatile i2c_master_state_t		i2cMasterState;
volatile spi_master_state_t		spiMasterState;
volatile spi_master_user_config_t	spiUserConfig;


/*
 *	TODO: move magic default numbers into constant definitions.
 */
volatile uint32_t			gWarpI2cBaudRateKbps	= 1;
volatile uint32_t			gWarpUartBaudRateKbps	= 1;
volatile uint32_t			gWarpSpiBaudRateKbps	= 1;
volatile uint32_t			gWarpSleeptimeSeconds	= 0;
volatile WarpModeMask			gWarpMode		= kWarpModeDisableAdcOnSleep;



void					lowPowerPinStates(void);
void					disableTPS82740A(void);
void					disableTPS82740B(void);
void					enableTPS82740A(uint16_t voltageMillivolts);
void					enableTPS82740B(uint16_t voltageMillivolts);
void					setTPS82740CommonControlLines(uint16_t voltageMillivolts);
void					printPinDirections(void);
void					dumpProcessorState(void);
void					enableSssupply(uint16_t voltageMillivolts);
void					disableSssupply(void);


/*
 *	From KSDK power_manager_demo.c <<BEGIN>>>
 */

clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
		break;
	}

	return result;
}


/*
 *	Power manager user callback
 */
power_manager_error_code_t callback0(power_manager_notify_struct_t *  notify,
					power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}





void
lowPowerPinStates(void)
{
	/*
	 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
	 *	we configure all pins as output and set them to a known state. We choose
	 *	to set them all to '0' since it happens that the devices we want to keep
	 *	deactivated (SI4705, PAN1326) also need '0'.
	 */

	/*
	 *			PORT A
	 */
	/*
	 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
	 */
	/*
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAsGpio);
	*/

	/*
	 *	PTA3 and PTA4 are the EXTAL/XTAL
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTA10 or PTA11
	 */

	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);



	/*
	 *			PORT B
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

	/*
	 *	PTB1 is connected to KL03_VDD. We have a choice of:
	 *		(1) Keep 'disabled as analog'.
	 *		(2) Set as output and drive high.
	 *
	 *	Pin state "disabled" means default functionality (ADC) is _active_
	 */
	if (gWarpMode & kWarpModeDisableAdcOnSleep)
	{
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
	}
	else
	{
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
	}

	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);

	/*
	 *	PTB3 and PTB3 (I2C pins) are true open-drain
	 *	and we purposefully leave them disabled.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);


	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB8 or PTB9
	 */

	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB12
	 */

	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAsGpio);



	/*
	 *	Now, set all the pins (except kWarpPinKL03_VDD_ADC, the SWD pins, and the XTAL/EXTAL) to 0
	 */



	/*
	 *	If we are in mode where we disable the ADC, then drive the pin high since it is tied to KL03_VDD
	 */
	if (gWarpMode & kWarpModeDisableAdcOnSleep)
	{
		GPIO_DRV_SetPinOutput(kWarpPinKL03_VDD_ADC);
	}

#ifdef WARP_FRDMKL03
	GPIO_DRV_ClearPinOutput(kWarpPinPAN1323_nSHUTD);
#else
	GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_nSHUTD);
#endif
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);
	GPIO_DRV_ClearPinOutput(kWarpPinCLKOUT32K);
	GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);

	/*
	 *	Drive these chip selects high since they are active low:
	 */
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);
	GPIO_DRV_SetPinOutput(kWarpPinADXL362_CS);

	/*
	 *	When the PAN1326 is installed, note that it has the
	 *	following pull-up/down by default:
	 *
	 *		HCI_RX / kWarpPinI2C0_SCL	: pull up
	 *		HCI_TX / kWarpPinI2C0_SDA	: pull up
	 *		HCI_RTS / kWarpPinSPI_MISO	: pull up
	 *		HCI_CTS / kWarpPinSPI_MOSI	: pull up
	 *
	 *	These I/Os are 8mA (see panasonic_PAN13xx.pdf, page 10),
	 *	so we really don't want to be driving them low. We
	 *	however also have to be careful of the I2C pullup and
	 *	pull-up gating. However, driving them high leads to
	 *	higher board power dissipation even when SSSUPPLY is off
	 *	by ~80mW on board #003 (PAN1326 populated).
	 *
	 *	In revB board, with the ISL23415 DCP pullups, we also
	 *	want I2C_SCL and I2C_SDA driven high since when we
	 *	send a shutdown command to the DCP it will connect
	 *	those lines to 25570_VOUT.
	 *
	 *	For now, we therefore leave the SPI pins low and the
	 *	I2C pins (PTB3, PTB4, which are true open-drain) disabled.
	 */

	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

	/*
	 *	HCI_RX / kWarpPinI2C0_SCL is an input. Set it low.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinI2C0_SCL);

	/*
	 *	HCI_TX / kWarpPinI2C0_SDA is an output. Set it high.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinI2C0_SDA);

	/*
	 *	HCI_RTS / kWarpPinSPI_MISO is an output. Set it high.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO);

	/*
	 *	From PAN1326 manual, page 10:
	 *
	 *		"When HCI_CTS is high, then CC256X is not allowed to send data to Host device"
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI);
}



//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************


//Software License Agreement (BSD License)
//
// Copyright (c) 2012, Adafruit Industries
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holders nor the
// names of its contributors may be used to endorse or promote products
// derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include "Adafruit_PN532.h"
unsigned char pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
unsigned char pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};// Uncomment these lines to enable debug output for PN532(SPI) and/or MIFARE related code
#define PN532_PACKBUFFSIZ 64
unsigned char pn532_packetbuffer[PN532_PACKBUFFSIZ];
#define _BV(bit) (1<<(bit))

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
void begin() {
    GPIO_DRV_ClearPinOutput(_ss);
    OSA_TimeDelay(1000);    // not exactly sure why but we have to send a dummy command to get synced up
    pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
    sendCommandCheckAck(pn532_packetbuffer, 1,1000);    // ignore response!    GPIO_DRV_SetPinOutput(_ss);
}
/**************************************************************************/
/*!
    @brief  Checks the firmware version of the PN5xx chip    @returns  The chip's firmware version and ID
*/
/**************************************************************************/
uint32_t getFirmwareVersion(void) {
  uint32_t response;  pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
  if (! sendCommandCheckAck(pn532_packetbuffer, 1,1000)) {
    return 0;
  }  // read data packet
  readdata(pn532_packetbuffer, 12);  // check some basic stuff
  if (0 != strncmp((const char *)pn532_packetbuffer, (const char *)pn532response_firmwarevers, 6)) {    return 0;
  }
  int offset = 6;  // Skip a response unsigned char when using I2C to ignore extra data.
  response = pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];  return response;
}
/**************************************************************************/
/*!
    @brief  Sends a command and waits a specified period for the ACK    @param  cmd       Pointer to the command buffer
    @param  cmdlen    The size of the command in unsigned chars
    @param  timeout   timeout before giving up    @returns  1 if everything is OK, 0 if timeout occured before an
              ACK was recieved
*/
/**************************************************************************/
// default timeout of one second
bool sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout) {
  //SEGGER_RTT_printf(0, "\r\nSending command %02X \n",cmd[0]);
  writecommand(cmd, cmdlen);  // Wait for chip to say its ready!
  if (!waitready(timeout)) {
    return false;
  }    // read acknowledgement
  if (!readack()) {
    return false;
  }  // For SPI only wait for the chip to be ready again.
  // This is unnecessary with I2C.
  if (!waitready(timeout)) {
    return false;
  }
  return true; // ack'd command
}/***** ISO14443A Commands ******//**************************************************************************/
/*!
    Waits for an ISO14443A target to enter the field    @param  cardBaudRate  Baud rate of the card
    @param  uid           Pointer to the array that will be populated
                          with the card's UID (up to 7 unsigned chars)
    @param  uidLength     Pointer to the variable that will hold the
                          length of the card's UID.    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool readPassiveTargetID(uint8_t cardbaudrate, uint8_t * uid, uint8_t * uidLength,uint16_t timeout) {
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
  pn532_packetbuffer[2] = cardbaudrate;
	if (!sendCommandCheckAck(pn532_packetbuffer, 3, timeout))
  {
    return 0x0;  // no cards read
  }  // wait for a card to enter the field (only possible with I2C)
  readdata(pn532_packetbuffer, 20);
	//SEGGER_RTT_printf(0, "\r\nGot me 20 bytes !");
  // check some basic stuff  /* ISO14443A card response should be in the following format:    unsigned char            Description
    // -------------   ------------------------------------------
    // b0..6           Frame header and preamble
    // b7              Tags Found
    // b8              Tag Number (only one used in this example)
    // b9..10          SENS_RES
    // b11             SEL_RES
    // b12             NFCID Length
    // b13..NFCIDLen   NFCID                                      */
  if (pn532_packetbuffer[7] != 1)
    return 0;
	uint16_t sens_res = pn532_packetbuffer[9];
  sens_res <<= 8;
  sens_res |= pn532_packetbuffer[10];
    /* Card appears to be Mifare Classic */
  *uidLength = pn532_packetbuffer[12];
	if (*uidLength>8){
		return 0x0;
	}
	//SEGGER_RTT_printf(0, "\r\n %d loops to do",*uidLength);
  for (uint8_t i=0; i < pn532_packetbuffer[12]; i++)
  {
		//SEGGER_RTT_printf(0, "\r\nDooing loop %d",i);
    uid[i] = pn532_packetbuffer[13+i];

  }
    return 1;
}



/************** high level communication functions (handles both I2C and SPI) */
/**************************************************************************/
/*!
    @brief  Tries to read the SPI or I2C ACK signal
*/
/**************************************************************************/
bool readack() {
  uint8_t ackbuff[6];
  readdata(ackbuff, 6);
  //SEGGER_RTT_printf(0, "\r\nACK was %02X %02X %02X %02X %02X %02X\n",ackbuff[0],ackbuff[1],ackbuff[2],ackbuff[3],ackbuff[4],ackbuff[5]);
  return (0 == strncmp((const char *)ackbuff, (const char *)pn532ack, 6));
}
/**************************************************************************/
/*!
    @brief  Return true if the PN532 is ready with a response.
*/
/**************************************************************************/
bool isready() {
    GPIO_DRV_ClearPinOutput(_ss);
    OSA_TimeDelay(2);
    spi_write(PN532_SPI_STATREAD);
    // read unsigned char
    uint8_t x = spi_read();    GPIO_DRV_SetPinOutput(_ss);
        // Check if status is ready.
    return x == PN532_SPI_READY;
}/**************************************************************************/
/*!
    @brief  Waits until the PN532 is ready.    @param  timeout   Timeout before giving up
*/
/**************************************************************************/
bool waitready(uint16_t timeout) {
  uint16_t timer = 0;
  while(!isready()) {
    if (timeout != 0) {
      timer += 10;
      if (timer > timeout) {
        return false;
      }
    }
    OSA_TimeDelay(10);
  }
  return true;
}/**************************************************************************/
/*!
    @brief  Reads n unsigned chars of data from the PN532 via SPI or I2C.    @param  buff      Pointer to the buffer where data will be written
    @param  n         Number of unsigned chars to be read
*/
/**************************************************************************/
void readdata(uint8_t* buff, uint8_t n) {

    GPIO_DRV_ClearPinOutput(_ss);
    OSA_TimeDelay(2);
    spi_write(PN532_SPI_DATAREAD);
    for (uint8_t i=0; i<n; i++) {
      OSA_TimeDelay(1);
      buff[i] = spi_read();

    }
    //SEGGER_RTT_printf(0, "\r\nReadDatais %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",buff[0],buff[1],buff[2],buff[3],buff[4],buff[5],buff[6],buff[7],buff[8],buff[9]);
    GPIO_DRV_SetPinOutput(_ss);
}/**************************************************************************/
/*!
    @brief  Writes a command to the PN532, automatically inserting the
            preamble and required frame details (checksum, len, etc.)    @param  cmd       Pointer to the command buffer
    @param  cmdlen    Command length in unsigned chars
*/
/**************************************************************************/
void writecommand(uint8_t* cmd, uint8_t cmdlen) {    // SPI command write.

    uint8_t checksum;
    cmdlen++;
    GPIO_DRV_ClearPinOutput(_ss);
    OSA_TimeDelay(2);     // or whatever the delay is for waking up the board
    spi_write(PN532_SPI_DATAWRITE);
    checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
    spi_write(PN532_PREAMBLE);
    spi_write(PN532_PREAMBLE);
    spi_write(PN532_STARTCODE2);    spi_write(cmdlen);
    spi_write(~cmdlen + 1);    spi_write(PN532_HOSTTOPN532);
    checksum += PN532_HOSTTOPN532;        for (uint8_t i=0; i<cmdlen-1; i++) {
      spi_write(cmd[i]);
      checksum += cmd[i];

    }
    spi_write(~checksum);
    spi_write(PN532_POSTAMBLE);
    GPIO_DRV_SetPinOutput(_ss);

}
/************** low level SPI *//**************************************************************************/
/*!
    @brief  Low-level SPI write wrapper    @param  c       8-bit command to write to the SPI bus
*/
/**************************************************************************/
void spi_write(uint8_t c) {    // Software SPI write.
    //SEGGER_RTT_printf(0, "\r\nSending command %02X \n",c);

    int8_t i;
    GPIO_DRV_SetPinOutput(_clk);
    for (i=0; i<8; i++) {
      GPIO_DRV_ClearPinOutput(_clk);
      if (c & _BV(i)) {
        GPIO_DRV_SetPinOutput(_mosi);
      } else {
        GPIO_DRV_ClearPinOutput(_mosi);
      }
      GPIO_DRV_SetPinOutput(_clk);
    }
}/**************************************************************************/
/*!
    @brief  Low-level SPI read wrapper    @returns The 8-bit value that was read from the SPI bus
*/
/**************************************************************************/
uint8_t spi_read(void) {

  int8_t i, x;
  x = 0;
    // Software SPI read.
    GPIO_DRV_SetPinOutput(_clk);
    OSA_TimeDelay(1);
    for (i=0; i<8; i++) {
      if (GPIO_DRV_ReadPinInput(_miso)) {
        //SEGGER_RTT_printf(0, "\r\nbit %d is 1",i);
        x |= _BV(i);
      }
      GPIO_DRV_ClearPinOutput(_clk);
      OSA_TimeDelay(1);
      GPIO_DRV_SetPinOutput(_clk);
      OSA_TimeDelay(1);
    }
  //  SEGGER_RTT_printf(0, "\r\nSpiReadReturned %02X",x);
  return x;
}

/**************************************************************************/
/*!
    @brief  Configures the SAM (Secure Access Module)
*/
/**************************************************************************/
bool SAMConfig(void) {
  pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
  pn532_packetbuffer[1] = 0x01; // normal mode;
  pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
  pn532_packetbuffer[3] = 0x01; // use IRQ pin!

  if (! sendCommandCheckAck(pn532_packetbuffer, 4,1000))
    return false;

  // read data packet
  readdata(pn532_packetbuffer, 8);

  int offset = 5;
  return  (pn532_packetbuffer[offset] == 0x15);
}



//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************


void carlWaitRelease(void) { //Debounce of keypad inputs to prevent multiple registers of single press.
	enum
	{
		_col1		= GPIO_MAKE_PIN(HW_GPIOA, 5),
		_col2		= GPIO_MAKE_PIN(HW_GPIOA, 7),
		_col3		= GPIO_MAKE_PIN(HW_GPIOA, 6),
	};

	while(GPIO_DRV_ReadPinInput(_col1)||GPIO_DRV_ReadPinInput(_col2)||GPIO_DRV_ReadPinInput(_col3)){
		OSA_TimeDelay(10);
	};
	return;
}

uint8_t
carlGetKey(void){ //For reading the keypad
	//Enum pins for easier use. Should do in header file, but here instead.
	enum
	{
		_row1		= GPIO_MAKE_PIN(HW_GPIOB, 5),
		_row2		= GPIO_MAKE_PIN(HW_GPIOB, 7),
		_row3		= GPIO_MAKE_PIN(HW_GPIOB, 6),
		_row4		= GPIO_MAKE_PIN(HW_GPIOB, 2),
		_col1		= GPIO_MAKE_PIN(HW_GPIOA, 5),
		_col2		= GPIO_MAKE_PIN(HW_GPIOA, 7),
		_col3		= GPIO_MAKE_PIN(HW_GPIOA, 6),
	};
	//set columns as inputs and set rows to 0.
	GPIO_DRV_SetPinDir(_col1,0);
	GPIO_DRV_SetPinDir(_col2,0);
	GPIO_DRV_SetPinDir(_col3,0);
	GPIO_DRV_ClearPinOutput(_row1);
	GPIO_DRV_ClearPinOutput(_row2);
	GPIO_DRV_ClearPinOutput(_row3);
	GPIO_DRV_ClearPinOutput(_row4);
	OSA_TimeDelay(10);
	//SEGGER_RTT_printf(0, "\r\nLooping\n");
while(1){ //While true loop with returns to escape.

	//SEGGER_RTT_printf(0, "."); //Debug, see how the program is looping.

	//multiplex rows and columns to work out which key is pressed.
	GPIO_DRV_SetPinOutput(_row1);
	OSA_TimeDelay(10);
	if (GPIO_DRV_ReadPinInput(_col1)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row1); return 1;}
	if (GPIO_DRV_ReadPinInput(_col2)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row1); return 2;}
	if (GPIO_DRV_ReadPinInput(_col3)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row1); return 3;}
	GPIO_DRV_ClearPinOutput(_row1);
	OSA_TimeDelay(10);
	GPIO_DRV_SetPinOutput(_row2);
	OSA_TimeDelay(10);
	if (GPIO_DRV_ReadPinInput(_col1)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row2); return 4;}
	if (GPIO_DRV_ReadPinInput(_col2)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row2); return 5;}
	if (GPIO_DRV_ReadPinInput(_col3)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row2); return 6;}
	GPIO_DRV_ClearPinOutput(_row2);
	OSA_TimeDelay(10);
	GPIO_DRV_SetPinOutput(_row3);
	OSA_TimeDelay(10);
	if (GPIO_DRV_ReadPinInput(_col1)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row3); return 7;}
	if (GPIO_DRV_ReadPinInput(_col2)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row3); return 8;}
	if (GPIO_DRV_ReadPinInput(_col3)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row3); return 9;}
	GPIO_DRV_ClearPinOutput(_row3);
	OSA_TimeDelay(10);
	//bottom row can allow for special commands, backspace etc.
	GPIO_DRV_SetPinOutput(_row4);
	OSA_TimeDelay(10);
	if (GPIO_DRV_ReadPinInput(_col1)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row4); return 10;}
	if (GPIO_DRV_ReadPinInput(_col2)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row4); return 11;}
	if (GPIO_DRV_ReadPinInput(_col3)){carlWaitRelease(); GPIO_DRV_ClearPinOutput(_row4); return 12;}
	GPIO_DRV_ClearPinOutput(_row4);
	OSA_TimeDelay(10);
}

}


bool
carlCheckKeys(uint8_t  key1,uint8_t  key2,uint8_t  key3,uint8_t  key4){
	// arguments - 4 keys. Take 4 presses from carlGetKey and compare. If match, return 1

	// Give UX via the yellow LED. Lots of repetition here. TODO: Make into a function/loop e.g.
	// for (i){keypresses[i]=getkeypress; YellowLED_off...}
	// for (i){if (expected_key[i]!=keypresses[i]){return 0} return 1}
	enum
	{
		red		= GPIO_MAKE_PIN(HW_GPIOB, 0),
		yellow= GPIO_MAKE_PIN(HW_GPIOB, 11),
		green	= GPIO_MAKE_PIN(HW_GPIOB, 10),
	};

  uint8_t key1_e=carlGetKey(); //Waits here until a key is pressed and released.
	GPIO_DRV_ClearPinOutput(yellow); //Turns off yellow LED.
	OSA_TimeDelay(100);	//waits 0.1s
	GPIO_DRV_SetPinOutput(yellow); //turns LED back on
	SEGGER_RTT_printf(0, "\r\n%d",key1_e); //debug ouput the keypress
	uint8_t key2_e=carlGetKey(); //repeat.
	GPIO_DRV_ClearPinOutput(yellow);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(yellow);
	SEGGER_RTT_printf(0, "%d",key2_e);
	uint8_t key3_e=carlGetKey();
	GPIO_DRV_ClearPinOutput(yellow);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(yellow);
	SEGGER_RTT_printf(0, "%d",key3_e);
	uint8_t key4_e=carlGetKey();
	GPIO_DRV_ClearPinOutput(yellow);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(yellow);
	SEGGER_RTT_printf(0, "%d",key4_e);
	if(key1==key1_e&&key2==key2_e&&key3==key3_e&&key4==key4_e){//If all the key inputs match, return 1.
		return 1;
	}

	return 0;
}


void
carlWhois(uint8_t * uid){//check to see if the user with that uid exists. If so, make sure they know their pin.

	enum
	{
		red		= GPIO_MAKE_PIN(HW_GPIOB, 0),
		yellow= GPIO_MAKE_PIN(HW_GPIOB, 11),
		green	= GPIO_MAKE_PIN(HW_GPIOB, 10),
	};
	//Storage for users.
	//const uint8_t validids[2][8]={{0x9B,0x33,0xE1,0x38,1,2,3,4},{0xD2,0xC3,0x6C,0x1B,1,1,1,1}};
	SEGGER_RTT_printf(0, "\r\nBefore validids\n");
	const uint8_t validids[50][8]={{212,28,110,19,48,31,225,49},{78,0,110,97,15,111,81,70},{132,208,125,146,59,192,9,133},{87,177,186,58,48,200,193,72},{27,120,196,117,209,220,184,101},{116,12,234,122,59,226,91,243},{60,94,187,8,148,242,46,210},{145,127,183,235,35,244,43,255},{209,40,27,34,7,225,208,21},{146,116,235,76,219,111,183,106},{48,91,58,34,70,98,187,63},{209,213,83,168,146,163,134,109},{103,228,231,7,185,199,232,184},{181,220,36,155,6,217,255,231},{112,79,63,77,249,47,78,112},{33,107,118,170,97,126,214,199},{252,19,51,34,245,236,230,51},{184,112,178,204,220,222,0,149},{68,235,132,199,27,52,214,24},{166,205,146,84,242,232,174,177},{25,36,219,25,246,74,247,128},{104,107,31,18,183,60,129,166},{14,119,104,15,174,43,187,174},{164,54,160,134,121,159,73,106},{168,67,242,215,219,10,205,194},{252,85,206,164,113,175,156,88},{187,203,71,255,17,120,242,26},{90,44,83,171,113,177,202,84},{224,199,0,195,205,152,203,240},{168,72,134,158,252,151,158,114},{82,194,131,127,230,76,212,200},{183,197,101,159,248,230,118,224},{108,116,236,107,188,61,107,196},{193,80,34,25,70,112,180,20},{79,50,48,225,100,50,208,249},{230,28,232,254,129,156,139,157},{30,48,123,213,100,220,8,95},{111,230,203,122,187,109,211,155},{194,255,176,148,54,225,114,206},{63,69,249,252,159,107,83,57},{73,238,4,239,122,222,158,233},{11,170,21,38,55,232,88,208},{22,79,140,215,230,233,9,158},{79,210,50,190,98,123,222,92},{244,150,58,118,220,84,166,188},{224,138,120,10,47,199,30,1},{24,234,246,84,49,70,76,57},{162,158,60,110,103,0,251,54},{0x9B,0x33,0xE1,0x38,1,2,3,4},{0xD2,0xC3,0x6C,0x1B,1,1,1,1}};
	SEGGER_RTT_printf(0, "\r\nAfter validids\n");
	for (uint8_t i=0;i<50;i++){ //Careful with the magic number. Should loop for each user present above.
					SEGGER_RTT_printf(0, "\r\nLoop %d\n",i);
		if(uid[0]==validids[i][0] && uid[1]==validids[i][1] && uid[2]==validids[i][2] && uid[3]==validids[i][3]){
			SEGGER_RTT_printf(0, "\r\nValid Card\n");
			GPIO_DRV_ClearPinOutput(red);
			GPIO_DRV_SetPinOutput(yellow);
			OSA_TimeDelay(50);
			if (carlCheckKeys(validids[i][4],validids[i][5],validids[i][6],validids[i][7])){ //If the user entered the correct pin.
				SEGGER_RTT_printf(0, "\r\nAccess granted\n");
				// UX via the LEDs. Green LED would control the door lock.
				GPIO_DRV_ClearPinOutput(yellow);
				GPIO_DRV_SetPinOutput(green);
				OSA_TimeDelay(5000);
				GPIO_DRV_ClearPinOutput(green);
				OSA_TimeDelay(1000);
			}else{
				SEGGER_RTT_printf(0, "\r\nIncorrect password \n");
			}
			return;
		}
	}


}

void
carlNFC(void){ // Reads from the PN532 module. When correct reads, pass off to other functions to check UID, PIN etc.
							 // Allows for easy testing - can pass arbitrary UIDs to the other functions to test them.
	enum
	{
		red		= GPIO_MAKE_PIN(HW_GPIOB, 0),
		yellow= GPIO_MAKE_PIN(HW_GPIOB, 11),
		green	= GPIO_MAKE_PIN(HW_GPIOB, 10),
	};
	//setup the LEDs as gpio
	PORT_HAL_SetMuxMode(PORTB_BASE, 11u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 10u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


// Print out keys, to test if carlGetKey is working.
// 	SEGGER_RTT_printf(0, "\r\n Reading Keys\n");
// for (uint8_t i=0; i<20; i++) {
// 	SEGGER_RTT_printf(0, "\r\n Read Key %d",carlGetKey());
//
// }

//Setup the PN532 pins as GPIO
PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAsGpio);
PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAsGpio);
GPIO_DRV_SetPinDir(_miso,0); //Set MISO as input


SEGGER_RTT_printf(0, "\r\nCarlNFCReadBegins2\n");

//These are to ensure that the PN532 is wired up correcly and has the expected firmware.
begin();
OSA_TimeDelay(10);
SEGGER_RTT_printf(0, "\r\nAbout to start versioncheck\n");
uint32_t status = getFirmwareVersion();
SEGGER_RTT_printf(0, "\r\nversion returned %d, expected 838927879\n",status);
OSA_TimeDelay(10);
SEGGER_RTT_printf(0, "\r\n\r\nSAMconfig returned %d, expected 1\n",SAMConfig());

while (1) {
	// Give user interface (Red LED on, others off)
	GPIO_DRV_SetPinOutput(red);
	GPIO_DRV_ClearPinOutput(yellow);
	GPIO_DRV_ClearPinOutput(green);
	//Setup required variables
	uint8_t success;
	uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
	uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)



	success = readPassiveTargetID(0, uid, &uidLength,1000); //returns 1 if a tag is read

	if (success) {
		//give debug output
		SEGGER_RTT_printf(0, "\r\nRead a tag !");
		SEGGER_RTT_printf(0, "\r\nUID length %d ",uidLength);
		SEGGER_RTT_printf(0, "\r\nUID %02X %02X %02X %02X ",uid[0],uid[1],uid[2],uid[3]);
		//Pass the UID to carlWhois
		carlWhois(uid);
}else{
	SEGGER_RTT_printf(0, "\r\nDidnt read a tag :(");
}
	OSA_TimeDelay(100);//delay before looping

}

}





void
disableTPS82740A(void)
{
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
}

void
disableTPS82740B(void)
{
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
}


void
enableTPS82740A(uint16_t voltageMillivolts)
{
	setTPS82740CommonControlLines(voltageMillivolts);
	GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);

	/*
	 *	Select the TS5A3154 to use the output of the TPS82740
	 *
	 *		IN = high selects the output of the TPS82740B:
	 *		IN = low selects the output of the TPS82740A:
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
}


void
enableTPS82740B(uint16_t voltageMillivolts)
{
	setTPS82740CommonControlLines(voltageMillivolts);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_SetPinOutput(kWarpPinTPS82740B_CTLEN);

	/*
	 *	Select the TS5A3154 to use the output of the TPS82740
	 *
	 *		IN = high selects the output of the TPS82740B:
	 *		IN = low selects the output of the TPS82740A:
	 */
	GPIO_DRV_SetPinOutput(kWarpPinTS5A3154_IN);
}


void
setTPS82740CommonControlLines(uint16_t voltageMillivolts)
{
	/*
	 *	 From Manual:
	 *
	 *		TPS82740A:	VSEL1 VSEL2 VSEL3:	000-->1.8V, 111-->2.5V
	 *		TPS82740B:	VSEL1 VSEL2 VSEL3:	000-->2.6V, 111-->3.3V
	 */

	switch(voltageMillivolts)
	{
		case 2600:
		case 1800:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

			break;
		}

		case 2700:
		case 1900:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

			break;
		}

		case 2800:
		case 2000:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

			break;
		}

		case 2900:
		case 2100:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

			break;
		}

		case 3000:
		case 2200:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

			break;
		}

		case 3100:
		case 2300:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

			break;
		}

		case 3200:
		case 2400:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

			break;
		}

		case 3300:
		case 2500:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

			break;
		}

		/*
		 *	Should never happen, due to previous check in enableSssupply()
		 */
		default:
		{
			SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
		}
	}


	/*
	 *	Vload ramp time of the TPS82740 is 800us max (datasheet, Section 8.5 / page 5)
	 */
	OSA_TimeDelay(1);
}



void
enableSssupply(uint16_t voltageMillivolts)
{
	if (voltageMillivolts >= 1800 && voltageMillivolts <= 2500)
	{
		enableTPS82740A(voltageMillivolts);
	}
	else if (voltageMillivolts >= 2600 && voltageMillivolts <= 3300)
	{
		enableTPS82740B(voltageMillivolts);
	}
	else
	{
		SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorInvalidVoltage RTT_CTRL_RESET "\n", voltageMillivolts);
	}
}



void
disableSssupply(void)
{
	disableTPS82740A();
	disableTPS82740B();

	/*
	 *	Clear the pin. This sets the TS5A3154 to use the output of the TPS82740B,
	 *	which shouldn't matter in any case. The main objective here is to clear
	 *	the pin to reduce power drain.
	 *
	 *		IN = high selects the output of the TPS82740B:
	 *		IN = low selects the output of the TPS82740A:
	 */
	GPIO_DRV_SetPinOutput(kWarpPinTS5A3154_IN);
}



int
main(void)
{
	//uint8_t					key;
	//WarpSensorDevice			menuTargetSensor = kWarpSensorADXL362;
	//uint16_t				menuI2cPullupValue = 32768;
	//uint8_t					menuRegisterAddress = 0x00;
	//uint16_t				menuSupplyVoltage = 0;


	//rtc_datetime_t				warpBootDate;

	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
							/*
							 *	NOTE: This order is depended on by POWER_SYS_SetMode()
							 *
							 *	See KSDK13APIRM.pdf Section 55.5.3
							 */
							&warpPowerModeWaitConfig,
							&warpPowerModeStopConfig,
							&warpPowerModeVlprConfig,
							&warpPowerModeVlpwConfig,
							&warpPowerModeVlpsConfig,
							&warpPowerModeVlls0Config,
							&warpPowerModeVlls1Config,
							&warpPowerModeVlls3Config,
							&warpPowerModeRunConfig,
						};

	WarpPowerManagerCallbackStructure			powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};



	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);



	/*
	 *	Setup board clock source.
	 */
	g_xtal0ClkFreq = 32768U;



	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();

	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);


	SEGGER_RTT_WriteString(0, "\n\n\n\rBooting Warp, in 3... ");
	OSA_TimeDelay(500);
	SEGGER_RTT_WriteString(0, "2...");
	OSA_TimeDelay(500);
	SEGGER_RTT_WriteString(0, "1...\n\r");
	OSA_TimeDelay(500);


	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM,
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);


	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));


	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;

	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(	&powerConfigs,
			sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
			&callbacks,
			sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
			);


	/*
	 *	Switch CPU to Very Low Power Run (VLPR) mode
	 */
	warpSetLowPowerMode(kWarpPowerModeVLPR, 0);


	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);

	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	lowPowerPinStates();

	/*
	 *	Toggle LED3 (kWarpPinSI4705_nRST)
	 */
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);


	/*
	 *	Initialization: the PAN1326, generating its 32k clock
	 */
	//Disable for now
	//initPAN1326B(&devicePAN1326BState);
#ifdef WARP_PAN1323ETU
	initPAN1323ETU(&devicePAN1323ETUState);
#endif


	disableSssupply();

OSA_TimeDelay(1000);

enableSssupply(2500);

	carlNFC();
	while (1)
	{
		SEGGER_RTT_WriteString(0, "\r[  \t\t\t\t    So very minimal, yet green.\t\t\t\t  ]\n\n");
		SEGGER_RTT_WaitKey();
	}
	return 0;
}
