using namespace std;
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <iostream>


#include "BBBGPIO.h"
#include "MCP23S17.h"
#include "Spi.h"
#include "max1300.h"

//SPIClass *SPIp

Max1300::Max1300()
{
	int a;
	ControlWord = 0;
	Channel = 0;

	Mode = 0;

	for(a=0;a<8;a++)
	{
		//init each ch to single ended and
		//DEFAULT SETTING
		//Single-Ended Bipolar -3 x VREFto +3 x VREF
		//FSR = 6 x VREF or +-12v
		AdcChConfig[a] = 0;
		AdcChRange[a] = 3;//SEPM12V;
	}
}

void Max1300::InitAdc(SPIClass *SPIp)
{
	//setup spi class pointer for access
	pSPI = SPIp;
	Mode = 0;
	WriteMode(0);
}


void Max1300::SetChannel(__u8 channel)
{
	Channel = (channel << 4) & 0x70;
}

void Max1300::SetChRange(__u8 channel,__u8 range)
{
	AdcChRange[channel] = range;
}

void Max1300::SetMode(__u8 mode)
{
	Mode = mode;
}

void Max1300::SetConfiguration(__u8 channel,__u8 config)
{
	AdcChConfig[channel] = config & 1;
}

__u32 Max1300::GetAdcChData(__u8 channel)
{
	__u32 retval = 0;
	//WriteMode(ch);
	//setup channel configuration and range
	//WriteConfiguration(channel);
	ControlWord = 0x80000000;
	channel &= 7;
	//set ch to convert
	ControlWord |= ((channel << 28));

	//send out controlword and recieve adc value
	retval = pSPI->SpiSendLong(ControlWord);
	//printf(" control = %X ", ControlWord);
	return retval;
}

void Max1300::GetAllAdcChData(__u32 *chdataarray)
{

}

void Max1300::WriteMode(__u8 mode)
{
	Mode = 0;
	Mode =( 0x88 | ((mode & 7) << 4));
	pSPI->SpiSendByte(Mode);

}

void Max1300::WriteConfiguration(__u8 channel)
{
	ConfigData = 0;
	ConfigData =( 0x80 | ((channel & 7) << 4));
	ConfigData |= ((AdcChConfig[channel] << 3) & 0x8);
	ConfigData |= (AdcChRange[channel]  & 0x7);
	//printf(" config = %.2X ", ConfigData);
	pSPI->SpiSendByte(ConfigData);
}


