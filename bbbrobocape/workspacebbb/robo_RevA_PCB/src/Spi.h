/*
 * spi.h
 *
 *  Created on: Oct 6, 2013
 *      Author: chris
 */
#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>



#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

class SPIClass
{
public:
int SpiInit(__u8 spiport);
void pabort(const char *s);

//static const char *device = "/dev/spidev1.0";
 __u8 mode;
 __u8 bits;
 __u32 speed;
 __u16 delay;
 __u8 cschange;
 __u32 TransferLength;
 int Spifd;

 __u8 SpiSendByte(__u8 value);
 __u16 SpiSendWord(__u32 value);
 __u32 SpiSend3Bytes(__u32 value);
 __u32 SpiSendLong(__u32 value);
 __u32 GetSpiClk(void);
 void SetSpiClk(__u32 clkrate);
 __u32 SpiSetClkMode(__u8 value);



void CloseSpi(void);
void SpiTransfer();

};

#endif /* SPI_H_ */

