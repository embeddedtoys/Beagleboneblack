/*
  MCP23S17.cpp  Version 0.1


  Features Implemented (by word and bit):
    I/O Direction
    Pull-up on/off
    Input inversion
    Output write
    Input read



  NOTE:  Addresses below are only valid when IOCON.BANK=0 (register addressing mode)
         This means one of the control register values can change register addresses!
         The default values is 0, so that's how we're using it.

         All registers except ICON (0xA and 0xB) are paired as A/B for each 8-bit GPIO port.
         Comments identify the port's name, and notes on how it is used.

         *THIS CLASS ENABLES THE ADDRESS PINS ON ALL CHIPS ON THE BUS WHEN THE FIRST CHIP OBJECT IS INSTANTIATED!

  USAGE: All Read/Write functions except wordWrite are implemented in two different ways.
         Individual pin values are set by referencing "pin #" and On/Off, Input/Output or High/Low where
         portA represents pins 0-7 and portB 8-15. So to set the most significant bit of portB, set pin # 15.
         To Read/Write the values for the entire chip at once, a word mode is supported buy passing a
         single argument to the function as 0x(portB)(portA). I/O mode Output is represented by 0.
         The wordWrite function was to be used internally, but was made public for advanced users to have
         direct and more efficient control by writing a value to a specific register pair.
*/

#include <linux/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include "Spi.h"               // SPI features
#include "MCP23S17.h"            // Header files for this class

// Defines to keep logical information symbolic go here

#define    HIGH          (1)
#define    LOW           (0)
#define    ON            (1)
#define    OFF           (0)
#define    OUTPUT        (0)
#define    INPUT         (1)


// Control byte and configuration register information - Control Byte: "0100 A2 A1 A0 R/W" -- W=0

#define    OPCODEW       (0b01000000)  // Opcode for MCP23S17 with LSB (bit0) set to write (0), address OR'd in later, bits 1-3
#define    OPCODER       (0b01000001)  // Opcode for MCP23S17 with LSB (bit0) set to read (1), address OR'd in later, bits 1-3
#define    ADDR_ENABLE   (0b00001000)  // Configuration register for MCP23S17, the only thing we change is enabling hardware addressing

// Constructor to instantiate an instance of MCP to a specific chip (address)

__u32  mtxd;
__u32  mrxd;

__u32 mxfrdata;



MCP::MCP(unsigned char address)
{
  _address     = address;
  _modeCache   = 0xFFFF;                // Default I/O mode is all input, 0xFFFF
  _outputCache = 0x0000;                // Default output state is all off, 0x0000
  _pullupCache = 0x0000;                // Default pull-up state is all off, 0x0000
  _invertCache = 0x0000;                // Default input inversion state is not inverted, 0x0000

};

// GENERIC BYTE WRITE - will write a byte to a register, arguments are register address and the value to write

void MCP::AddressEnable(SPIClass *SPIp)
{
	pSPI = SPIp;

	byteWrite(IOCON, ADDR_ENABLE);

}

void MCP::byteWrite(__u8 reg, __u8 value) {      // Accept the register and byte
    mxfrdata = 0;
	mxfrdata = (OPCODEW | (_address << 1));
	mxfrdata <<=8;
	mxfrdata |=reg;
	mxfrdata <<=8;
	mxfrdata|=value;

	pSPI->SpiSetClkMode(SPI_MODE_0);

	pSPI->SpiSend3Bytes(mxfrdata);

}

// GENERIC WORD WRITE - will write a word to a register pair, LSB to first register, MSB to next higher value register 

void MCP::wordWrite(__u8 reg, unsigned int word) {  // Accept the start register and word
    mxfrdata = 0;
	mxfrdata = (OPCODEW | (_address << 1));
	mxfrdata <<=8;
	mxfrdata |=reg;
	mxfrdata <<=16;
	mxfrdata|=word;

	pSPI->SpiSetClkMode(SPI_MODE_0);

	pSPI->SpiSendLong(mxfrdata);

}

// MODE SETTING FUNCTIONS - BY PIN AND BY WORD

void MCP::pinMode(__u8 pin, __u8 mode) {  // Accept the pin # and I/O mode
  if ((pin < 1) | (pin > 16))
	  {
	  return;               // If the pin value is not valid (1-16) return, do nothing and return
	  }
  if (mode == INPUT) {                          // Determine the mode before changing the bit state in the mode cache
    _modeCache |= 1 << (pin - 1);               // Since input = "HIGH", OR in a 1 in the appropriate place
  } else {
    _modeCache &= ~(1 << (pin - 1));            // If not, the mode must be output, so and in a 0 in the appropriate place
  }
  wordWrite(IODIRA, _modeCache);                // Call the generic word writer with start register and the mode cache
}

void MCP::pinMode(unsigned int mode) {    // Accept the word�
  wordWrite(IODIRA, mode);                // Call the the generic word writer with start register and the mode cache
  _modeCache = mode;

}

// THE FOLLOWING WRITE FUNCTIONS ARE NEARLY IDENTICAL TO THE FIRST AND ARE NOT INDIVIDUALLY COMMENTED

// WEAK PULL-UP SETTING FUNCTIONS - BY WORD AND BY PIN

void MCP::pullupMode(__u8 pin, __u8 mode) {
  if ((pin < 1) | (pin > 16))
	  {
	  return;
	  }
  if (mode == ON) {
    _pullupCache |= 1 << (pin - 1);
  } else {
    _pullupCache &= ~(1 << (pin -1));
  }
  wordWrite(GPPUA, _pullupCache);
}


void MCP::pullupMode(unsigned int mode) { 
  wordWrite(GPPUA, mode);
  _pullupCache = mode;
}


// INPUT INVERSION SETTING FUNCTIONS - BY WORD AND BY PIN

void MCP::inputInvert(__u8 pin, __u8 mode) {
	if ((pin < 1) | (pin > 16))
		  {
		  return;
		  }
  if (mode == ON) {
    _invertCache |= 1 << (pin - 1);
  } else {
    _invertCache &= ~(1 << (pin - 1));
  }
  wordWrite(IPOLA, _invertCache);
}

void MCP::inputInvert(unsigned int mode) { 
  wordWrite(IPOLA, mode);
  _invertCache = mode;
}


// WRITE FUNCTIONS - BY WORD AND BY PIN

void MCP::digitalWrite(__u8 pin, __u8 value) {
	if ((pin < 1) | (pin > 16))
		  {
		  return;
		  }
  if (value) {
    _outputCache |= 1 << (pin - 1);
  } else {
    _outputCache &= ~(1 << (pin - 1));
  }
  wordWrite(GPIOA, _outputCache);
}

void MCP::digitalWrite(unsigned int value) { 
  wordWrite(GPIOA, value);
  _outputCache = value;
}


// READ FUNCTIONS - BY WORD, BYTE AND BY PIN

unsigned int MCP::digitalRead(void) {       // This function will read all 16 bits of I/O, and return them as a word in the format 0x(portB)(portA)
    unsigned int value = 0;                   // Initialize a variable to hold the read values to be returned

    mxfrdata = 0;
  	mxfrdata = (OPCODER | (_address << 1));
  	mxfrdata <<=8;
  	mxfrdata |=GPIOA;
  	mxfrdata <<=16;

  	pSPI->SpiSetClkMode(SPI_MODE_0);

  	value = pSPI->SpiSendLong(mxfrdata);

  	return value;                             // Return the constructed word, the format is 0x(portB)(portA)
}

__u8 MCP::byteRead(__u8 reg) {        // This function will read a single register, and return it
  __u8 value = 0;                        // Initialize a variable to hold the read values to be returned
  __u32 retval;

    mxfrdata = 0;
  	mxfrdata = (OPCODER | (_address << 1));
  	mxfrdata <<=8;
  	mxfrdata |=reg;
  	mxfrdata <<=8;
  	mxfrdata|=value;

  	pSPI->SpiSetClkMode(SPI_MODE_0);

  	retval = pSPI->SpiSend3Bytes(mxfrdata);
    value = (__u8)retval & 0xff;
  return value;                             // Return the constructed word, the format is 0x(register value)
}

__u8 MCP::digitalRead(__u8 pin) {
	// Return a single bit value, supply the necessary bit (1-16)
	if ((pin < 1) | (pin > 16))
		  {
		  return 0;
		  }
	// If the pin value is not valid (1-16) return, do nothing and return
    return digitalRead() & (1 << (pin - 1)) ? HIGH : LOW;  // Call the word reading function, extract HIGH/LOW information from the requested pin
}






