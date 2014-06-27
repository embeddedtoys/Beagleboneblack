/*
 * spi.cpp
 *
 *  Created on: Oct 6, 2013
 *      Author: chris
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "Spi.h"

using namespace std;

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

void SPIClass::pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev1.0";

__u8 txd[32];

__u8 rxd[ARRAY_SIZE(txd)] = {0, };

__u32 SPIClass::GetSpiClk()
{
	return speed;
}

void SPIClass::SetSpiClk(__u32 clkrate)
{
	speed = clkrate;
}


void SPIClass::SpiTransfer()
{
	int ret;


	struct spi_ioc_transfer tr;

		tr.tx_buf = (unsigned long)txd,
		tr.rx_buf = (unsigned long)rxd,
		tr.len = TransferLength,
		tr.delay_usecs = delay,
		tr.speed_hz = speed,
		tr.bits_per_word = bits,
		tr.cs_change = cschange;

		//printf("made it\n");
	ret = ioctl(Spifd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
	{
		pabort("can't send spi message again");
	}


}

__u8 SPIClass::SpiSendByte(__u8 value)
{
	txd[0] = value;
	TransferLength = sizeof(__u8);
	bits = 8;

	SpiTransfer();

	return rxd[0];
}

__u16 SPIClass::SpiSendWord(__u32 value)
{
	__u16 retval;
	txd[1] = (__u8)((value >> 8) & 0xff);
	txd[0] = (__u8)(value & 0xff);
	TransferLength = sizeof(__u16);
	bits = 16;
	cschange = 0;
	SpiTransfer();

	retval = rxd[0];
	retval<<=8;
	retval |= rxd[1];

	return retval;
}


__u32 SPIClass::SpiSend3Bytes(__u32 value)
{
	__u32 retval;
	txd[2] = (__u8)((value >> 16) & 0xff);
	txd[1] = (__u8)((value >> 8) & 0xff);
	txd[0] = (__u8)(value & 0xff);
	TransferLength = sizeof(__u32);
	bits = 24;

	//cschange = 0;
	SpiTransfer();

	retval =0;
	retval = rxd[0];
	retval<<=8;
	retval |= rxd[1];
	retval<<=8;
	retval |= rxd[2];

	return retval;
}

__u32 SPIClass::SpiSendLong(__u32 value)
{
	__u32 retval;

	txd[3] = (__u8)((value >> 24) & 0xff);
	txd[2] = (__u8)((value >> 16) & 0xff);
	txd[1] = (__u8)((value >> 8) & 0xff);
	txd[0] = (__u8)(value & 0xff);
	TransferLength = sizeof(__u32);
	bits = 32;

	SpiTransfer();

	retval =0;
	retval = (__u32)rxd[3];
	retval<<=8;
	retval |= (__u32)rxd[2];
	retval<<=8;
	retval |= (__u32)rxd[1];
	retval<<=8;
	retval |= (__u32)rxd[0];

	//for (ret = 0; ret < 4; ret++)
	//	{
				//if (!(ret % 6))	puts("");
				//printf("%.2X ", retval);//rxd[ret]);
	//	}

	return retval;
}


int SPIClass::SpiInit(__u8 spiport)
{
	int ret = 0;

	if(spiport == 0)
	{
		Spifd = open("/dev/spidev2.0", O_RDWR);
	}

	else if(spiport == 1)
	{
		Spifd = open("/dev/spidev1.0", O_RDWR);
	}


	if (Spifd < 0)
	{
		pabort("can't open device");
	}

	mode = 0;
	bits = 8;
	speed = 4500000;
	delay = 0;
	cschange = 1;

	ret = ioctl(Spifd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
	{
		pabort("can't set spi mode");
	}

	ret = ioctl(Spifd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
	{
		pabort("can't get spi mode");
	}

	/*
	 * bits per word
	 */
	ret = ioctl(Spifd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(Spifd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(Spifd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(Spifd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	return ret;
}

void SPIClass::CloseSpi(void)
{
	close(Spifd);
}

__u32 SPIClass::SpiSetClkMode(__u8 value)
{
	int ret = 0;

	mode = value;

	ret = ioctl(Spifd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
	{
		pabort("can't set spi mode");
	}

	ret = ioctl(Spifd, SPI_IOC_RD_MODE, &mode);

	if (ret == -1)
	{
		pabort("can't get spi mode");
	}
	return ret;
}


