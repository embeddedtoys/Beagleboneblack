/*
 * SimpleGPIO.cpp
 *
 * Modifications by Derek Molloy, School of Electronic Engineering, DCU
 * www.derekmolloy.ie
 * Almost entirely based on Software by RidgeRun:
 *
 * Copyright (c) 2011, RidgeRun
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the RidgeRun.
 * 4. Neither the name of the RidgeRun nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY RIDGERUN ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL RIDGERUN BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "BBBGPIO.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include "BBBGPIO.h"
#include "MCP23S17.h"
#include "Spi.h"
#include "max1300.h"



unsigned int GpioNumber[16]=
{
	//AdcStrbgpion I
		60,
	//AdcCsn O
		30,
	//MotInt1gpion I
		45,
	//DigIOCsn O
		31,
	//StepClk0gpion O
		44,
	//StepClk1gpion O
		27,
	//StepClk2gpion O
		46,
	//StepClk3gpion O
		61,
	//MotDir0gpion O
		7,
	//MotDir1gpion O
		26,
	//MotDir2gpion O
		47,
	//MotDir3gpion O
		65,
	//SpiDevA0gpion O
		48,
	//SpiDevA1gpion O
		49,
	//SpiDevA2gpion O
		115,
	//
		114
};



PIN_DIRECTION GpioDirCon[16]=
{
	//AdcStrbgpion
		INPUT_PIN,
	//AdcCsn O
	    OUTPUT_PIN,
	//MotInt1gpion
		INPUT_PIN,
	//DigIOCsn O
		OUTPUT_PIN,
	//StepClk0gpion
		OUTPUT_PIN,
	//StepClk1gpion
		OUTPUT_PIN,
	//StepClk2gpion
		OUTPUT_PIN,
	//StepClk3gpion
		OUTPUT_PIN,
	//MotDir0gpion
		INPUT_PIN,
	//MotDir1gpion
		INPUT_PIN,
	//MotDir2gpion
		INPUT_PIN,
	//MotDir3gpion
		INPUT_PIN,
	//SpiDevA0gpion
		OUTPUT_PIN,
	//SpiDevA1gpion
		OUTPUT_PIN,
	//SpiDevA2gpion
		OUTPUT_PIN,
	//
		INPUT_PIN
};


/*

unsigned int GpioNumber[27]=
{
	//	gpio0.2	    out
		2,
	//	gpio0.3	    out
		3,
	//	gpio0.7     out
		7,
	//	gpio0.14	out
		14,
	//	gpio0.15	out
		15,
	//	gpio0.22	out
		22,
	//	gpio0.23	out
		23,
	//	gpio0.26	out
		26,
	//	gpio0.27	out
		27,
	//	gpio0.30	out
		30,
	//	gpio0.31	out
		31,
	//	gpio1.12	out
		44,
	//	gpio1.13	out
		45,
	//	gpio1.14	out
		46,
	//	gpio1.15	out
		47,
	//	gpio1.16	out
		48,
	//	gpio1.18	out
		50,
	//	gpio1.19	out
		51,
	//	gpio1.28	out
		60,
	//	gpio1.29	out
		61,
	//	gpio2.1 	out
		65,
	//	gpio2.2 	out
		66,
	//	gpio2.3 	out
		67,
	//	gpio2.4 	out
		68,
	//	gpio2.5 	out
		69,
	//	gpio3.19	out
		115,
	//	gpio3.21	out
		117
};


PIN_DIRECTION GpioDirCon[27]=
{
		//gpio0.2	out
		OUTPUT_PIN,
		//gpio0.3	out
		OUTPUT_PIN,
		//gpio0.7   out
		OUTPUT_PIN,
		//gpio0.14	out
		OUTPUT_PIN,
		//gpio0.15	out
		OUTPUT_PIN,
		//gpio0.22	out
		OUTPUT_PIN,
		//gpio0.23	out
		OUTPUT_PIN,
		//gpio0.26	out
		OUTPUT_PIN,
		//gpio0.27	out
		OUTPUT_PIN,
		//gpio0.30	out
		INPUT_PIN,
		//gpio0.31	out
		INPUT_PIN,
		//gpio1.12	out
		INPUT_PIN,
		//gpio1.13	out
		INPUT_PIN,
		//gpio1.14	out
		OUTPUT_PIN,
		//gpio1.15	out
		OUTPUT_PIN,
		//gpio1.16	out
		OUTPUT_PIN,
		//gpio1.18	out
		OUTPUT_PIN,
		//gpio1.19	out
		OUTPUT_PIN,
		//gpio1.28	out
		OUTPUT_PIN,
		//gpio1.29	out
		OUTPUT_PIN,
		//gpio2.1 	out
		OUTPUT_PIN,
		//gpio2.2 	out
		OUTPUT_PIN,
		//gpio2.3 	out
		OUTPUT_PIN,
		//gpio2.4 	out
		OUTPUT_PIN,
		//gpio2.5 	out
		OUTPUT_PIN,
		//gpio3.19 	out
		OUTPUT_PIN,
		//gpio3.21 	out
		OUTPUT_PIN,
};
*/

//INPUT_PIN
void BBBGPIO::InitBBBGpio()
{
	ExportBBBGPIOPins();
	//SetSpiCs(NoCsn);

}

void BBBGPIO::SetSpiCs(unsigned int Csn)
{

	if(Csn < 6)
	{
		//spics a0
		if(Csn&1)
		{
			gpio_set_value(SpiDevA0gpio, HIGH);
		}
		else
		{
			gpio_set_value(SpiDevA0gpio, LOW);
		}

		//spics a1
		if(Csn&2)
		{
			gpio_set_value(SpiDevA1gpio, HIGH);
		}
		else
		{
			gpio_set_value(SpiDevA1gpio, LOW);
		}

		//spics a2
		if(Csn&4)
		{
			gpio_set_value(SpiDevA2gpio, HIGH);
		}
		else
		{
			gpio_set_value(SpiDevA2gpio, LOW);
		}
	}

	else
	{
		if(Csn == 6)
		{
			;
		}

		if(Csn == 7)
		{
			;
		}
	}




}

void BBBGPIO::ExportBBBGPIOPins()
{
	int index;

	for(index = 0; index < 16; index++)
		{
			gpio_export(GpioNumber[index]);
			gpio_set_dir(GpioNumber[index], GpioDirCon[index]);
		}
}

void BBBGPIO::UnexportBBBGPIOPins()
{
	int index;
	for(index = 0; index < 16; index++)
	{
		gpio_unexport(GpioNumber[index]);   // The push button switch
	}
}
/****************************************************************
 * gpio_export
 ****************************************************************/
int BBBGPIO::gpio_export(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

	return 0;
}

/****************************************************************
 * gpio_unexport
 ****************************************************************/
int BBBGPIO::gpio_unexport(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int BBBGPIO::gpio_set_dir(unsigned int gpio, PIN_DIRECTION out_flag)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/direction");
		return fd;
	}

	if (out_flag == OUTPUT_PIN)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);

	close(fd);
	return 0;
}

/****************************************************************
 * gpio_set_value
 ****************************************************************/
int BBBGPIO::gpio_set_value(unsigned int gpio, PIN_VALUE value)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
		return fd;
	}

	if (value==LOW)
		write(fd, "0", 2);
	else
		write(fd, "1", 2);

	close(fd);
	return 0;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int BBBGPIO::gpio_get_value(unsigned int gpio, unsigned int *value)
{
	int fd;
	char buf[MAX_BUF];
	char ch;

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}

	read(fd, &ch, 1);

	if (ch != '0') {
		*value = 1;
	} else {
		*value = 0;
	}

	close(fd);
	return 0;
}


/****************************************************************
 * gpio_set_edge
 ****************************************************************/

int BBBGPIO::gpio_set_edge(unsigned int gpio, char *edge)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-edge");
		return fd;
	}

	write(fd, edge, strlen(edge) + 1);
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_fd_open
 ****************************************************************/

int BBBGPIO::gpio_fd_open(unsigned int gpio)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY | O_NONBLOCK );
	if (fd < 0) {
		perror("gpio/fd_open");
	}
	return fd;
}

/****************************************************************
 * gpio_fd_close
 ****************************************************************/

int BBBGPIO::gpio_fd_close(int fd)
{
	return close(fd);
}

BBBGPIO::~BBBGPIO()
{
	UnexportBBBGPIOPins();

}
