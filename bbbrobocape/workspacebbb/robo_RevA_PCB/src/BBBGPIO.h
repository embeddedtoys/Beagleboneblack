/*
 * SimpleGPIO.h
 *
 * Copyright Derek Molloy, School of Electronic Engineering, Dublin City University
 * www.derekmolloy.ie
 *
 * Based on Software by RidgeRun
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

#ifndef BBBGPIO_H_
#define BBBGPIO_H_

 /****************************************************************
 * Constants
 ****************************************************************/
#include <linux/types.h>
#include "BBBGPIO.h"
#include "MCP23S17.h"
#include "Spi.h"
#include "max1300.h"


#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define MAX_BUF 64

#define NoCsn	    0
#define Motor1Csn	1
#define Motor2Csn	2
#define Motor3Csn	3
#define Motor4Csn	4
#define StatusCsn	5
#define DioCsn	    6
#define AdcCsn	    7

enum PIN_DIRECTION{
	INPUT_PIN=0,
	OUTPUT_PIN=1
};

enum PIN_VALUE{
	LOW=0,
	HIGH=1
};




#define 		AdcStrbgpio		60
#define 		AdcCsngpio		30
#define 		DigIOCsngpio	31
#define 		MotInt1gpio		45

#define 		StepClk0gpio	44
#define 		StepClk1gpio	27
#define 		StepClk2gpio	46
#define 		StepClk3gpio	61
#define 		MotDir0gpio		7
#define 		MotDir1gpio		26
#define 		MotDir2gpio		47
#define 		MotDir3gpio		65
#define 		SpiDevA0gpio	48
#define 		SpiDevA1gpio	49
#define 		SpiDevA2gpio	115
#define         GPIO318			114

#define			gpio1_15	47
#define			gpio1_16	48
#define			gpio1_17	49

/****************************************************************
 * gpio_class
 ****************************************************************/
class BBBGPIO
{
public:

	void InitBBBGpio(void);
	void SetSpiCs(unsigned int Csn);
	void ExportBBBGPIOPins(void);
	void UnexportBBBGPIOPins(void);
	int gpio_export(unsigned int gpio);
	int gpio_unexport(unsigned int gpio);
	int gpio_set_dir(unsigned int gpio, PIN_DIRECTION out_flag);
	int gpio_set_value(unsigned int gpio, PIN_VALUE value);
	int gpio_get_value(unsigned int gpio, unsigned int *value);
	int gpio_set_edge(unsigned int gpio, char *edge);
	int gpio_fd_open(unsigned int gpio);
	int gpio_fd_close(int fd);
	~BBBGPIO();
};

#endif /* SIMPLEGPIO_H_ */
