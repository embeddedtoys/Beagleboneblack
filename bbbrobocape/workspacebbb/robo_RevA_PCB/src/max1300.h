/*
 * max1300.h
 *
 *  Created on: Oct 7, 2013
 *      Author: chris
 */

#ifndef MAX1300_H_
#define MAX1300_H_

#include <linux/types.h>

#include "Spi.h"
//Start Bit. The first logic 1 after CS
//goes low defines the beginning of the analog input configuration byte.
//7 START

//Channel-Select Bits. SEL[2:0] select the analog input channel to be configured

//6  C2
//5  C1
//4  C0

//Differential or Single-Ended Configuration Bit. DIF/SGL= 0 configures the selected analog input channel
//for single-ended operation. DIF/SGL= 1 configures the channel for differential operation. In single-ended
//mode, input voltages are measured between the selected input channel and AGND1, as shown in
//Table 4. In differential mode, the input voltages are measured between two input channels, as shown in
//Table 5. Be aware that changing DIF/SGLadjusts the FSR, as shown in Table 6.
//3 DIF/SGL

//Input-Range-Select Bits. R[2:0] select the input voltage range, as shown in Table 6 and Figure 7.
//2R2
//1R1
//0R0

//VREF = 4.096V
//Single-Ended Bipolar (-3 x VREF)/4 to (+3 x VREF)/4
//Full-Scale Range (FSR) = (3 x VREF)/2
#define SEPM3V		1

//Single-Ended Unipolar (-3 x VREF)/2 to 0
//FSR = (3 x VREF)/2
#define SEM6V		2

//Single-Ended Unipolar 0 to (+3 x VREF)/2
//FSR = (+3 x VREF)/2
#define SEP6V		3

//Single-Ended Bipolar (-3 x VREF)/2 to (+3 x VREF)/2
//FSR = 3 x VREF
#define SEPM6V		4

//Single-Ended Unipolar -3 x VREFto 0
//FSR = 3 x VREF
#define SEM12V		5

//Single-Ended Unipolar 0 to +3 x VREF
//FSR = 3 x VREF
#define SEP12V		6

//DEFAULT SETTING
//Single-Ended Bipolar -3 x VREFto +3 x VREF
//FSR = 6 x VREF
#define SEPM12V		7

//Differential Bipolar (-3 x VREF)/2 to (+3 x VREF)/2
//FSR = 3 x VREF
#define DIFFPM3V	1

//Differential Bipolar -3 x VREFto +3 x VREF
//FSR = 6 x VREF
#define DIFFPM6V	4

//Differential Bipolar -6 x VREFto +6 x VREF
//FSR = 12 x VREF
#define DIFFPM12V	7
#define NumOfADCCh 8

class Max1300
{
public:

	__u32 ControlWord;
	__u8 Mode;
	__u8 Channel;
	__u8 ConfigData;
	__u32 ChannelData[NumOfADCCh];
	SPIClass *pSPI;
	__u8 AdcChConfig[NumOfADCCh];
	__u8 AdcChRange[NumOfADCCh];
	Max1300();
	void SetChannel(__u8 channel);
	void SetChRange(__u8 channel,__u8 range);
	void SetMode(__u8 mode);
	void SetConfiguration(__u8 channel,__u8 config);
	void WriteMode(__u8 channel);
	void WriteConfiguration(__u8 channel);
	__u32 GetAdcChData(__u8);
	void GetAllAdcChData(__u32 *chdataarray);
	void InitAdc(SPIClass *SPIp);
};

#endif /* MAX1300_H_ */
