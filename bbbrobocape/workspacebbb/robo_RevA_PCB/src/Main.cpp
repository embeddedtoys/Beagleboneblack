/*

 */
/*
#include <iostream>
#include <string>
#include <unistd.h>
#include "BBBGPIO.h"
#include "MCP23S17.h"
#include "Spi.h"

*/

/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */
using namespace std;
#include <iconv.h>

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
#include <time.h>
#include <pthread.h>

#include <sys/mman.h>
#include <errno.h>
#include <string.h>




#include <ctime>
#include <string>
#include <curses.h>


//using namespace std;

#include "socket.h"





#include "PWM.h"
#include "BBBGPIO.h"
#include "MCP23S17.h"
#include "Spi.h"
#include "max1300.h"
#include "motorcontrol.h"
#include "ad5293.h"
#include "ad9910dds.h"

/* dSPIN overcurrent threshold options */
typedef	enum {
    dSPIN_OCD_TH_375mA		=((uint8_t)0x00),
 	dSPIN_OCD_TH_750mA		=((uint8_t)0x01),
 	dSPIN_OCD_TH_1125mA		=((uint8_t)0x02),
 	dSPIN_OCD_TH_1500mA		=((uint8_t)0x03),
 	dSPIN_OCD_TH_1875mA		=((uint8_t)0x04),
 	dSPIN_OCD_TH_2250mA		=((uint8_t)0x05),
 	dSPIN_OCD_TH_2625mA		=((uint8_t)0x06),
 	dSPIN_OCD_TH_3000mA		=((uint8_t)0x07),
 	dSPIN_OCD_TH_3375mA		=((uint8_t)0x08),
 	dSPIN_OCD_TH_3750mA		=((uint8_t)0x09),
 	dSPIN_OCD_TH_4125mA		=((uint8_t)0x0A),
	dSPIN_OCD_TH_4500mA		=((uint8_t)0x0B),
 	dSPIN_OCD_TH_4875mA		=((uint8_t)0x0C),
 	dSPIN_OCD_TH_5250mA		=((uint8_t)0x0D),
 	dSPIN_OCD_TH_5625mA		=((uint8_t)0x0E),
 	dSPIN_OCD_TH_6000mA		=((uint8_t)0x0F)
} dSPIN_OCD_TH_TypeDef;

/* dSPIN STEP_MODE register masks */
typedef enum {
	dSPIN_STEP_MODE_STEP_SEL		=((uint8_t)0x07),
	dSPIN_STEP_MODE_SYNC_SEL		=((uint8_t)0x70),
	dSPIN_STEP_MODE_SYNC_EN			=((uint8_t)0x80)
} dSPIN_STEP_MODE_Masks_TypeDef;

 /* dSPIN STEP_MODE register options */
/* dSPIN STEP_SEL options */
typedef enum {
	dSPIN_STEP_SEL_1		=((uint8_t)0x00),
	dSPIN_STEP_SEL_1_2		=((uint8_t)0x01),
	dSPIN_STEP_SEL_1_4		=((uint8_t)0x02),
	dSPIN_STEP_SEL_1_8		=((uint8_t)0x03),
	dSPIN_STEP_SEL_1_16		=((uint8_t)0x04),
	dSPIN_STEP_SEL_1_32		=((uint8_t)0x05),
	dSPIN_STEP_SEL_1_64		=((uint8_t)0x06),
	dSPIN_STEP_SEL_1_128	=((uint8_t)0x07)
} dSPIN_STEP_SEL_TypeDef;

/* dSPIN SYNC_SEL options */
typedef enum {
	dSPIN_SYNC_SEL_1_2		=((uint8_t)0x00),
	dSPIN_SYNC_SEL_1		=((uint8_t)0x10),
	dSPIN_SYNC_SEL_2		=((uint8_t)0x20),
	dSPIN_SYNC_SEL_4		=((uint8_t)0x30),
	dSPIN_SYNC_SEL_8		=((uint8_t)0x40),
	dSPIN_SYNC_SEL_16		=((uint8_t)0x50),
	dSPIN_SYNC_SEL_32		=((uint8_t)0x60),
	dSPIN_SYNC_SEL_64		=((uint8_t)0x70)
} dSPIN_SYNC_SEL_TypeDef;

#define dSPIN_SYNC_EN		0x80

/* dSPIN ALARM_EN register options */
typedef enum {
	dSPIN_ALARM_EN_OVERCURRENT			=((uint8_t)0x01),
	dSPIN_ALARM_EN_THERMAL_SHUTDOWN		=((uint8_t)0x02),
	dSPIN_ALARM_EN_THERMAL_WARNING		=((uint8_t)0x04),
	dSPIN_ALARM_EN_UNDER_VOLTAGE		=((uint8_t)0x08),
	dSPIN_ALARM_EN_STALL_DET_A			=((uint8_t)0x10),
	dSPIN_ALARM_EN_STALL_DET_B			=((uint8_t)0x20),
	dSPIN_ALARM_EN_SW_TURN_ON			=((uint8_t)0x40),
	dSPIN_ALARM_EN_WRONG_NPERF_CMD		=((uint8_t)0x80)
} dSPIN_ALARM_EN_TypeDef;

/* dSPIN Config register masks */
typedef enum {
	dSPIN_CONFIG_OSC_SEL					=((uint16_t)0x0007),
	dSPIN_CONFIG_EXT_CLK					=((uint16_t)0x0008),
	dSPIN_CONFIG_SW_MODE					=((uint16_t)0x0010),
	dSPIN_CONFIG_EN_VSCOMP					=((uint16_t)0x0020),
	dSPIN_CONFIG_OC_SD						=((uint16_t)0x0080),
	dSPIN_CONFIG_POW_SR						=((uint16_t)0x0300),
	dSPIN_CONFIG_F_PWM_DEC					=((uint16_t)0x1C00),
	dSPIN_CONFIG_F_PWM_INT					=((uint16_t)0xE000)
} dSPIN_CONFIG_Masks_TypeDef;

/* dSPIN Config register options */
typedef enum {
	dSPIN_CONFIG_INT_16MHZ					=((uint16_t)0x0000),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_2MHZ		=((uint16_t)0x0008),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_4MHZ		=((uint16_t)0x0009),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_8MHZ		=((uint16_t)0x000A),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_16MHZ		=((uint16_t)0x000B),
	dSPIN_CONFIG_EXT_8MHZ_XTAL_DRIVE		=((uint16_t)0x0004),
	dSPIN_CONFIG_EXT_16MHZ_XTAL_DRIVE		=((uint16_t)0x0005),
	dSPIN_CONFIG_EXT_24MHZ_XTAL_DRIVE		=((uint16_t)0x0006),
	dSPIN_CONFIG_EXT_32MHZ_XTAL_DRIVE		=((uint16_t)0x0007),
	dSPIN_CONFIG_EXT_8MHZ_OSCOUT_INVERT		=((uint16_t)0x000C),
	dSPIN_CONFIG_EXT_16MHZ_OSCOUT_INVERT	=((uint16_t)0x000D),
	dSPIN_CONFIG_EXT_24MHZ_OSCOUT_INVERT	=((uint16_t)0x000E),
	dSPIN_CONFIG_EXT_32MHZ_OSCOUT_INVERT	=((uint16_t)0x000F)
} dSPIN_CONFIG_OSC_MGMT_TypeDef;

typedef enum {
	dSPIN_CONFIG_SW_HARD_STOP		=((uint16_t)0x0000),
	dSPIN_CONFIG_SW_USER			=((uint16_t)0x0010)
} dSPIN_CONFIG_SW_MODE_TypeDef;

typedef enum {
	dSPIN_CONFIG_VS_COMP_DISABLE	=((uint16_t)0x0000),
	dSPIN_CONFIG_VS_COMP_ENABLE		=((uint16_t)0x0020)
} dSPIN_CONFIG_EN_VSCOMP_TypeDef;

typedef enum {
	dSPIN_CONFIG_OC_SD_DISABLE		=((uint16_t)0x0000),
	dSPIN_CONFIG_OC_SD_ENABLE		=((uint16_t)0x0080)
} dSPIN_CONFIG_OC_SD_TypeDef;

typedef enum {
	dSPIN_CONFIG_SR_180V_us		=((uint16_t)0x0000),
	dSPIN_CONFIG_SR_290V_us		=((uint16_t)0x0200),
	dSPIN_CONFIG_SR_530V_us		=((uint16_t)0x0300)
} dSPIN_CONFIG_POW_SR_TypeDef;

typedef enum {
	dSPIN_CONFIG_PWM_DIV_1		=(((uint16_t)0x00)<<13),
	dSPIN_CONFIG_PWM_DIV_2		=(((uint16_t)0x01)<<13),
	dSPIN_CONFIG_PWM_DIV_3		=(((uint16_t)0x02)<<13),
	dSPIN_CONFIG_PWM_DIV_4		=(((uint16_t)0x03)<<13),
	dSPIN_CONFIG_PWM_DIV_5		=(((uint16_t)0x04)<<13),
	dSPIN_CONFIG_PWM_DIV_6		=(((uint16_t)0x05)<<13),
	dSPIN_CONFIG_PWM_DIV_7		=(((uint16_t)0x06)<<13)
} dSPIN_CONFIG_F_PWM_INT_TypeDef;

typedef enum {
	dSPIN_CONFIG_PWM_MUL_0_625		=(((uint16_t)0x00)<<10),
	dSPIN_CONFIG_PWM_MUL_0_75		=(((uint16_t)0x01)<<10),
	dSPIN_CONFIG_PWM_MUL_0_875		=(((uint16_t)0x02)<<10),
	dSPIN_CONFIG_PWM_MUL_1			=(((uint16_t)0x03)<<10),
	dSPIN_CONFIG_PWM_MUL_1_25		=(((uint16_t)0x04)<<10),
	dSPIN_CONFIG_PWM_MUL_1_5		=(((uint16_t)0x05)<<10),
	dSPIN_CONFIG_PWM_MUL_1_75		=(((uint16_t)0x06)<<10),
	dSPIN_CONFIG_PWM_MUL_2			=(((uint16_t)0x07)<<10)
} dSPIN_CONFIG_F_PWM_DEC_TypeDef;

/* Status Register bit masks */
typedef enum {
	dSPIN_STATUS_HIZ			=(((uint16_t)0x0001)),
	dSPIN_STATUS_BUSY			=(((uint16_t)0x0002)),
	dSPIN_STATUS_SW_F			=(((uint16_t)0x0004)),
	dSPIN_STATUS_SW_EVN			=(((uint16_t)0x0008)),
	dSPIN_STATUS_DIR			=(((uint16_t)0x0010)),
	dSPIN_STATUS_MOT_STATUS		=(((uint16_t)0x0060)),
	dSPIN_STATUS_NOTPERF_CMD	=(((uint16_t)0x0080)),
	dSPIN_STATUS_WRONG_CMD		=(((uint16_t)0x0100)),
	dSPIN_STATUS_UVLO			=(((uint16_t)0x0200)),
	dSPIN_STATUS_TH_WRN			=(((uint16_t)0x0400)),
	dSPIN_STATUS_TH_SD			=(((uint16_t)0x0800)),
	dSPIN_STATUS_OCD			=(((uint16_t)0x1000)),
	dSPIN_STATUS_STEP_LOSS_A	=(((uint16_t)0x2000)),
	dSPIN_STATUS_STEP_LOSS_B	=(((uint16_t)0x4000)),
	dSPIN_STATUS_SCK_MOD		=(((uint16_t)0x8000))
} dSPIN_STATUS_Masks_TypeDef;

/* Status Register options */
typedef enum {
	dSPIN_STATUS_MOT_STATUS_STOPPED			=(((uint16_t)0x0000)<<13),
	dSPIN_STATUS_MOT_STATUS_ACCELERATION	=(((uint16_t)0x0001)<<13),
	dSPIN_STATUS_MOT_STATUS_DECELERATION	=(((uint16_t)0x0002)<<13),
	dSPIN_STATUS_MOT_STATUS_CONST_SPD		=(((uint16_t)0x0003)<<13)
} dSPIN_STATUS_TypeDef;

/* dSPIN internal register addresses */
typedef enum {
	dSPIN_ABS_POS			=((uint8_t)0x01),
	dSPIN_EL_POS			=((uint8_t)0x02),
	dSPIN_MARK				=((uint8_t)0x03),
	dSPIN_SPEED				=((uint8_t)0x04),
	dSPIN_ACC				=((uint8_t)0x05),
	dSPIN_DEC				=((uint8_t)0x06),
	dSPIN_MAX_SPEED			=((uint8_t)0x07),
	dSPIN_MIN_SPEED			=((uint8_t)0x08),
	dSPIN_FS_SPD			=((uint8_t)0x15),
	dSPIN_KVAL_HOLD			=((uint8_t)0x09),
	dSPIN_KVAL_RUN			=((uint8_t)0x0A),
	dSPIN_KVAL_ACC			=((uint8_t)0x0B),
	dSPIN_KVAL_DEC			=((uint8_t)0x0C),
	dSPIN_INT_SPD			=((uint8_t)0x0D),
	dSPIN_ST_SLP			=((uint8_t)0x0E),
	dSPIN_FN_SLP_ACC		=((uint8_t)0x0F),
	dSPIN_FN_SLP_DEC		=((uint8_t)0x10),
	dSPIN_K_THERM			=((uint8_t)0x11),
	dSPIN_ADC_OUT			=((uint8_t)0x12),
	dSPIN_OCD_TH			=((uint8_t)0x13),
	dSPIN_STALL_TH			=((uint8_t)0x14),
	dSPIN_STEP_MODE			=((uint8_t)0x16),
	dSPIN_ALARM_EN			=((uint8_t)0x17),
	dSPIN_CONFIG			=((uint8_t)0x18),
	dSPIN_STATUS			=((uint8_t)0x19),
	dSPIN_RESERVED_REG1		=((uint8_t)0x1A),
	dSPIN_RESERVED_REG2		=((uint8_t)0x1B)
} dSPIN_Registers_TypeDef;

/* dSPIN command set */
typedef enum {
	dSPIN_NOP			=((uint8_t)0x00),
	dSPIN_SET_PARAM		=((uint8_t)0x00),
	dSPIN_GET_PARAM		=((uint8_t)0x20),
	dSPIN_RUN			=((uint8_t)0x50),
	dSPIN_STEP_CLOCK	=((uint8_t)0x58),
	dSPIN_MOVE			=((uint8_t)0x40),
	dSPIN_GO_TO			=((uint8_t)0x60),
	dSPIN_GO_TO_DIR		=((uint8_t)0x68),
	dSPIN_GO_UNTIL		=((uint8_t)0x82),
	dSPIN_RELEASE_SW	=((uint8_t)0x92),
	dSPIN_GO_HOME		=((uint8_t)0x70),
	dSPIN_GO_MARK		=((uint8_t)0x78),
	dSPIN_RESET_POS		=((uint8_t)0xD8),
	dSPIN_RESET_DEVICE	=((uint8_t)0xC0),
	dSPIN_SOFT_STOP		=((uint8_t)0xB0),
	dSPIN_HARD_STOP		=((uint8_t)0xB8),
	dSPIN_SOFT_HIZ		=((uint8_t)0xA0),
	dSPIN_HARD_HIZ		=((uint8_t)0xA8),
	dSPIN_GET_STATUS	=((uint8_t)0xD0),
	dSPIN_RESERVED_CMD1	=((uint8_t)0xEB),
	dSPIN_RESERVED_CMD2	=((uint8_t)0xF8)
} dSPIN_Commands_TypeDef;


/* dSPIN direction options */
	typedef enum {
		FWD		=((uint8_t)0x01),
		REV		=((uint8_t)0x00)
	} dSPIN_Direction_TypeDef;


//instantiate classes for interfacing to the hardware

//digital i/o from bbb based on sys/whatever
BBBGPIO BBBGpio;
BBBGPIO *pBBBGpio;

//mcp23s17 16 bit port used for the motor i/o
MCP MotorGPIO(4);
MCP *pMotorGPIO;

//mcp23s17 16bit port used for the digital i/o
MCP DigitalGPIO(5);

//spi class io
SPIClass SPI;
SPIClass SPI0;

//pointer to spi class to pass to other classes
SPIClass *pSpi;
SPIClass *pSpi0;

//max1300 adc class
Max1300 Max1300adc;

//pwm class

const long periodNS = 20 * MILLISECONDS_TO_NANOSECONDS;
PWM::Pin pinD("P8_13", periodNS); // Since both pins share the same channel, they're periods must be the same
PWM::Pin pinC("P8_19", periodNS);
PWM::Pin pinA("P9_14", periodNS); // Since both pins share the same channel, they're periods must be the same
PWM::Pin pinB("P9_16", periodNS);

//L6470 motor controller class
L6470Motor Motor1;
L6470Motor Motor2;
L6470Motor Motor3;
L6470Motor Motor4;

L6470Motor *pMotor1;
L6470Motor *pMotor2;
L6470Motor *pMotor3;
L6470Motor *pMotor4;

L6470Motor *pCurrentMotor;
L6470Motor *pMotorSel;

//Ad9910 DDS9910;

int spwma = 0;
int spwmb = 0;
int spwmc = 0;
int spwmd = 0;


void GetAdcChannelData(__u8 ch);
void InitHW(void);
void InitAllMotors(void);
void TestAllMotors(void);
void SetUpPWM(void);

//void TestPwm(void);
void DisplayRxData(void);
void *ADCThread(void *ptr);
void ProcessMotorCommand(void);
void ServeItUp(void);
void SetMotorRegister(void);
void ProbeMotors(void);
void WhileMotorsBusy(void);
void SetPeripheralRegData(void);
void SetPWMDutyCycle(void);
void ProcessMotorCmdXfr(void);
void ProcessMotorRegXfr(void);

__u32 adcdata[64];

typedef enum
{
	EMotorNop = 0,
	ESetParam,
	EGetParam,
	ERun,
	EStepClock,
	EMove,
	EGoTo,
	EGoToDir,
	EGoUntil,
	EReleaseSw,
	EGoHome,
	EGoMark,
	EResetPos,
	EResetDevice,
	ESoftStop,
	EHardStop,
	ESoftHIZ,
	EHardHiZ,
	EGetStatus

}EMotorCommand;


typedef struct
{
	int32_t PacketType;
	int32_t PwmA;
	int32_t PwmB;
	int32_t PwmC;
	int32_t PwmD;
	int32_t PortADir;
	int32_t PortBDir;
	int32_t PortADataIn;
	int32_t PortADataOut;
	int32_t PortBDataIn;
	int32_t PortBDataOut;
	int32_t Ain0Range;
	int32_t Ain1Range;
	int32_t Ain2Range;
	int32_t Ain3Range;
	int32_t Ain4Range;
	int32_t Ain5Range;
	int32_t Ain6Range;
	int32_t Ain7Range;
	int32_t Ain0Data;
	int32_t Ain1Data;
	int32_t Ain2Data;
	int32_t Ain3Data;
	int32_t Ain4Data;
	int32_t Ain5Data;
	int32_t Ain6Data;
	int32_t Ain7Data;
}TPeripheralData;

TPeripheralData PeripheralData;

typedef struct
{
	int32_t PacketType;
	int32_t Command;
	int32_t Param1;
	int32_t Param2;
	int32_t Param3;
	int32_t Param4;
	int32_t MotorSel;
}TMotorCommandData;

TMotorCommandData MotorCmdData;

typedef struct
{
  uint32_t MotorSelect;
  uint32_t ABS_POS;
  uint32_t EL_POS;
  uint32_t MARK;
  uint32_t SPEED;
  uint32_t ACC;
  uint32_t DEC;
  uint32_t MAX_SPEED;
  uint32_t MIN_SPEED;
  uint32_t FS_SPD;
  uint32_t KVAL_HOLD;
  uint32_t KVAL_RUN;
  uint32_t KVAL_ACC;
  uint32_t KVAL_DEC;
  uint32_t INT_SPD;
  uint32_t ST_SLP;
  uint32_t FN_SLP_ACC;
  uint32_t FN_SLP_DEC;
  uint32_t K_THERM;
  uint32_t ADC_OUT;
  uint32_t OCD_TH;
  uint32_t STALL_TH;
  uint32_t STEP_MODE;
  uint32_t ALARM_EN;
  uint32_t CONFIG;
}TMotor_RegsStruct_TypeDef;

TMotor_RegsStruct_TypeDef Motor_RegsStruct;


/* struct to hold data to be passed to a thread
   this shows how multiple data items can be passed to a thread */

#define PeripheralRegData 	0x7a550000
#define PWMData 			0x7a550003
//#define
#define MotorCommandXfr	 	0x7a550002

#define AdcReadXfr			0x7a550005
#define MotorRegisterXfr    0x7a550006

int PabMode = 0xffff;
unsigned int PabData=0;
pthread_t ThreadID;
int MotorsInstalled = 0;
int MotorCmdExecuting = 0;
__u32 MotorRegSel = 1;

char buffer[256];




int main()
{
	Motor_RegsStruct.MotorSelect = 1;
	InitHW();
	ServeItUp();

	while(1)
	{
		BBBGpio.gpio_set_value(DigIOCsngpio, LOW);

		DigitalGPIO.digitalWrite(PabData++);
		PabData&=0xffff;

		BBBGpio.gpio_set_value(DigIOCsngpio, HIGH);

		usleep(10000);
	}

	return 0;
}


void ServeItUp()
{

	int len;
	int a;
	int  iret1 = 0;

	//PWM::Pin pinA("P8_13", periodNS); // Since both pins share the same channel, they're periods must be the same
	//PWM::Pin pinB("P8_19", periodNS);
	//PWM::Pin pinC("P9_14", periodNS); // Since both pins share the same channel, they're periods must be the same
	//PWM::Pin pinD("P9_16", periodNS);

	iret1 = pthread_create( &ThreadID, NULL, ADCThread, NULL);

	NL::init();

	cout << "\nStarting server...";
	cout.flush();

	NL::Socket* clientConnection = NULL;

	try {

		NL::Socket server(5000);

		clientConnection = server.accept();
		void * ptr;

		buffer[255] = '\0';

		while((len=clientConnection->read(buffer,255)))
		{
			//DisplayRxData();
			memcpy(&PeripheralData, &buffer, sizeof(TPeripheralData));

			if(PeripheralData.PacketType == PeripheralRegData)
			{
			SetPeripheralRegData();

			}

			if(PeripheralData.PacketType == PWMData)
			{
			//printf("\npwm cmd");
				SetPWMDutyCycle();

			}

			if(PeripheralData.PacketType == MotorCommandXfr)
			{
				ProcessMotorCmdXfr();

			}

			if(PeripheralData.PacketType == MotorRegisterXfr)
			{
				ProcessMotorRegXfr();

			}


			if(PeripheralData.PacketType == AdcReadXfr)
			{
				//printf("msel = %d\n",PeripheralData.PwmA);
				MotorRegSel = PeripheralData.PwmA;
				ptr = &adcdata[0];

				clientConnection->send(ptr,120);
			}
		}

		delete clientConnection;

		cout << "\nClient disconnected. Exit...";

	}

	catch(NL::Exception e)
	{
		cout << "\n***Error*** " << e.what();

		if(clientConnection != NULL)
		{
			delete clientConnection;
		}
	}

	pthread_cancel(ThreadID);

}


void ProcessMotorRegXfr()
{
	int a;

	memcpy(&Motor_RegsStruct, &buffer, sizeof(Motor_RegsStruct));
	MotorCmdExecuting = 1;
	while(MotorCmdExecuting!=2);

	if((Motor_RegsStruct.MotorSelect & 8) == 8)
	{
		for(a = 1; a < 5; a++)
		{
			Motor_RegsStruct.MotorSelect = a;
			SetMotorRegister();
		}

	}

	else
	{
		SetMotorRegister();
	}
	MotorCmdExecuting = 0;
}

void ProcessMotorCmdXfr()
{
	int a;
	memcpy(&MotorCmdData, &buffer, sizeof(MotorCmdData));
	if(MotorCmdData.Command >= 0 && MotorCmdData.Command <= 18)
	{
		MotorCmdExecuting = 1;

		while(MotorCmdExecuting!=2);

		if((MotorCmdData.MotorSel & 8) == 8)
		{
			for(a = 1; a < 5; a++)
			{
				MotorCmdData.MotorSel = a;
				ProcessMotorCommand();
				//usleep(5000);
				if(MotorCmdData.Command == EResetDevice)
				{
					switch(a)
					{
					case 1://printf("\nresetting motor 1");
						   Motor1.ResetL6470();
						   Motor1.SetupL6470();
						   Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
						   //printf("\nMotor1 Status=%.4X", Motor1.dSPIN_rx_data);

						   break;

					case 2:Motor2.ResetL6470();
					       //printf("\nresetting motor 2");
						   Motor2.SetupL6470();
						   Motor2.dSPIN_rx_data = Motor2.dSPIN_Get_Status();
						   //printf("\nMotor2 Status=%.4X", Motor2.dSPIN_rx_data);

						   break;

					case 3:Motor3.ResetL6470();
					       //printf("\nresetting motor 3");
						   Motor3.SetupL6470();
						   Motor3.dSPIN_rx_data = Motor3.dSPIN_Get_Status();
						   //printf("\nMotor3 Status=%.4X", Motor3.dSPIN_rx_data);

						   break;

					case 4:Motor4.ResetL6470();
					       //printf("\nresetting motor 4");
						   Motor4.SetupL6470();
						   Motor4.dSPIN_rx_data = Motor4.dSPIN_Get_Status();
						   //printf("\nMotor4 Status=%.4X\n", Motor4.dSPIN_rx_data);

						   break;
					}

				}

				//resetting l6470 causes flag to go low.
				//reading status sets it back hi
				Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
				//printf("\nMotor1 Status=%.4X", Motor1.dSPIN_rx_data);
				Motor2.dSPIN_rx_data = Motor2.dSPIN_Get_Status();
				//printf("\nMotor2 Status=%.4X", Motor2.dSPIN_rx_data);
				Motor3.dSPIN_rx_data = Motor3.dSPIN_Get_Status();
				//printf("\nMotor3 Status=%.4X", Motor3.dSPIN_rx_data);
				Motor4.dSPIN_rx_data = Motor4.dSPIN_Get_Status();
				//printf("\nMotor4 Status=%.4X\n", Motor4.dSPIN_rx_data);

			}
		}

		else
		{
			ProcessMotorCommand();
		}

		MotorCmdExecuting = 0;

	}
}


void SetPWMDutyCycle()
{
	pinA.SetDutyUS(PeripheralData.PwmA+1500);
	pinB.SetDutyUS(PeripheralData.PwmB+1500);
	pinC.SetDutyUS(PeripheralData.PwmC+1500);
	pinD.SetDutyUS(PeripheralData.PwmD+1500);
}

void SetPeripheralRegData()
{

	int pdata;

	pinA.SetDutyUS(PeripheralData.PwmA+1500);
	pinB.SetDutyUS(PeripheralData.PwmB+1500);
	pinC.SetDutyUS(PeripheralData.PwmC+1500);
	pinD.SetDutyUS(PeripheralData.PwmD+1500);


	Max1300adc.AdcChRange[0] = PeripheralData.Ain0Range;
	Max1300adc.AdcChRange[1] = PeripheralData.Ain1Range;
	Max1300adc.AdcChRange[2] = PeripheralData.Ain2Range;
	Max1300adc.AdcChRange[3] = PeripheralData.Ain3Range;
	Max1300adc.AdcChRange[4] = PeripheralData.Ain4Range;
	Max1300adc.AdcChRange[5] = PeripheralData.Ain5Range;
	Max1300adc.AdcChRange[6] = PeripheralData.Ain6Range;
	Max1300adc.AdcChRange[7] = PeripheralData.Ain7Range;
	//printf("range = %d\n",Max1300adc.AdcChRange[1]);

	//BBBGpio.SetSpiCs(DioCsn);


	PabMode = PeripheralData.PortADir & 0xff;
	PabMode <<= 8;
	PabMode |= (PeripheralData.PortBDir & 0xff);
	BBBGpio.gpio_set_value(DigIOCsngpio, LOW);
	DigitalGPIO.pinMode(PabMode);
	BBBGpio.gpio_set_value(DigIOCsngpio, HIGH);

	PabData = PeripheralData.PortADataOut & 0xff;
	PabData <<= 8;
	PabData |= (PeripheralData.PortBDataOut & 0xff);
	BBBGpio.gpio_set_value(DigIOCsngpio, LOW);
	DigitalGPIO.digitalWrite(PabData);
	BBBGpio.gpio_set_value(DigIOCsngpio, HIGH);

	BBBGpio.gpio_set_value(DigIOCsngpio, LOW);
	pdata = DigitalGPIO.digitalRead();
	BBBGpio.gpio_set_value(DigIOCsngpio, HIGH);
	//BBBGpio.SetSpiCs(0);

	adcdata[8] = ((pdata >> 8) & 0xff);
	adcdata[9] = ((pdata) & 0xff);

}

void SetMotorRegister()
{
	//make sure the right motor is selected
	switch(Motor_RegsStruct.MotorSelect)
		{
		case 1: pMotorSel = pMotor1;

				break;

		case 2: pMotorSel = pMotor2;
				break;

		case 3: pMotorSel = pMotor3;
				break;

		case 4: pMotorSel = pMotor4;
				break;

		default : break;

		}

	/* Acceleration rate , range 14.55 to 59590 steps/s2 */
	pMotorSel->dSPIN_RegsStruct.ACC 		= Motor_RegsStruct.ACC;
	/* Deceleration rate , range 14.55 to 59590 steps/s2 */
	pMotorSel->dSPIN_RegsStruct.DEC 		= Motor_RegsStruct.DEC;
	/* Maximum speed , range 15.25 to 15610 steps/s */
	pMotorSel->dSPIN_RegsStruct.MAX_SPEED 	= Motor_RegsStruct.MAX_SPEED;
	/* Minimum speed , range 0 to 976.3 steps/s */
	pMotorSel->dSPIN_RegsStruct.MIN_SPEED	= Motor_RegsStruct.MIN_SPEED;
	/* Full step speed , range 7.63 to 15625 steps/s */
	pMotorSel->dSPIN_RegsStruct.FS_SPD 	= Motor_RegsStruct.FS_SPD;
	/* Hold duty cycle (torque) , range 0 to 99.6% */
	pMotorSel->dSPIN_RegsStruct.KVAL_HOLD 	= Motor_RegsStruct.KVAL_HOLD;
	/* Run duty cycle (torque) , range 0 to 99.6% */
	pMotorSel->dSPIN_RegsStruct.KVAL_RUN 	= Motor_RegsStruct.KVAL_RUN;
	/* Acceleration duty cycle (torque) , range 0 to 99.6% */
	pMotorSel->dSPIN_RegsStruct.KVAL_ACC 	= Motor_RegsStruct.KVAL_ACC;
	/* Deceleration duty cycle (torque) , range 0 to 99.6% */
	pMotorSel->dSPIN_RegsStruct.KVAL_DEC 	= Motor_RegsStruct.KVAL_DEC;
	/* Intersect speed settings for BEMF compensation , range 0 to 3906 steps/s */
	pMotorSel->dSPIN_RegsStruct.INT_SPD 	= Motor_RegsStruct.INT_SPD;
	/* BEMF start slope settings for BEMF compensation , range 0 to 0.4% s/step */
	pMotorSel->dSPIN_RegsStruct.ST_SLP 	= Motor_RegsStruct.ST_SLP;
	/* BEMF final acc slope settings for BEMF compensation , range 0 to 0.4% s/step */
	pMotorSel->dSPIN_RegsStruct.FN_SLP_ACC = Motor_RegsStruct.FN_SLP_ACC;
	/* BEMF final dec slope settings for BEMF compensation , range 0 to 0.4% s/step */
	pMotorSel->dSPIN_RegsStruct.FN_SLP_DEC = Motor_RegsStruct.FN_SLP_DEC;
	/* Thermal compensation param settings , range 1 to 1.46875 */
	pMotorSel->dSPIN_RegsStruct.K_THERM 	= Motor_RegsStruct.K_THERM;
	/* Overcurrent threshold  */
	pMotorSel->dSPIN_RegsStruct.OCD_TH 	= Motor_RegsStruct.OCD_TH;
	/* Stall threshold settings , range 31.25 to 4000mA */
	pMotorSel->dSPIN_RegsStruct.STALL_TH 	= Motor_RegsStruct.STALL_TH;
	/* Step mode settings  */
	pMotorSel->dSPIN_RegsStruct.STEP_MODE 	= Motor_RegsStruct.STEP_MODE;
	/* Alarm settings  */
	pMotorSel->dSPIN_RegsStruct.ALARM_EN 	= Motor_RegsStruct.ALARM_EN;
	/* Internal oscillator,  OSCOUT clock, supply voltage compensation , *
	 * overcurrent shutdown , slew-rate , PWM frequency    */
	pMotorSel->dSPIN_RegsStruct.CONFIG 	= Motor_RegsStruct.CONFIG;

}


void ProcessMotorCommand()
{
	//make sure the right motor is selected

	switch(MotorCmdData.MotorSel)
	{
	case 1: pCurrentMotor = pMotor1;
			break;

	case 2: pCurrentMotor = pMotor2;
			break;

	case 3: pCurrentMotor = pMotor3;
			break;

	case 4: pCurrentMotor = pMotor4;
			break;

	default : break;

	}

	//printf("\nCommand = %X\n",MotorCmdData.Command);

	while(pCurrentMotor->dSPIN_Busy_HW())
			{
				;
			}


	switch(MotorCmdData.Command)
	{
						//void dSPIN_Nop(void);
	case    EMotorNop:	pCurrentMotor->dSPIN_Nop();
						break;

						//dSPIN_Set_Param(__u8 param, __u32 value);
	case  	ESetParam:	pCurrentMotor->dSPIN_Set_Param((MotorCmdData.Param1 & 0xff), MotorCmdData.Param2);
						break;

						//uint32_t dSPIN_Get_Param(__u8 param);
	case  	EGetParam:  pCurrentMotor->dSPIN_Get_Param((MotorCmdData.Param1 & 0xff));
						break;

						//void dSPIN_Run(uint8_t direction, uint32_t speed);
	case  	ERun:		pCurrentMotor->dSPIN_Run((MotorCmdData.Param1 & 0xff), MotorCmdData.Param2);
						break;

						//void dSPIN_Step_Clock(uint8_t direction);
	case  	EStepClock: pCurrentMotor->dSPIN_Step_Clock((MotorCmdData.Param1 & 0xff));
						break;

						//void dSPIN_Move(uint8_t direction, uint32_t n_step);
	case  	EMove:		pCurrentMotor->dSPIN_Move((MotorCmdData.Param1 & 0xff), MotorCmdData.Param2);
						break;

						//void dSPIN_Go_To(uint32_t abs_pos);
	case  	EGoTo:		pCurrentMotor->dSPIN_Go_To(MotorCmdData.Param1);
						break;

						//void dSPIN_Go_To_Dir(uint8_t direction, uint32_t abs_pos);
	case  	EGoToDir:	pCurrentMotor->dSPIN_Go_To_Dir((MotorCmdData.Param1 & 0xff), MotorCmdData.Param2);
						break;

						//void dSPIN_Go_Until(uint8_t action, uint8_t direction, uint32_t speed);
	case  	EGoUntil:	pCurrentMotor->dSPIN_Go_Until((MotorCmdData.Param1 & 0xff),(MotorCmdData.Param2 & 0xff), (MotorCmdData.Param3 & 0xff));
						break;

						//void dSPIN_Release_SW(uint8_t action, uint8_t direction);
	case  	EReleaseSw:	pCurrentMotor->dSPIN_Release_SW((MotorCmdData.Param1 & 0xff), (MotorCmdData.Param2 & 0xff));
						break;

						//void dSPIN_Go_Home(void);
	case  	EGoHome:	pCurrentMotor->dSPIN_Go_Home();
						break;

						//void dSPIN_Go_Mark(void);
	case  	EGoMark:	pCurrentMotor->dSPIN_Go_Mark();
						break;

						//void dSPIN_Reset_Pos(void);
	case  	EResetPos:	pCurrentMotor->dSPIN_Reset_Pos();
						break;

						//void dSPIN_Reset_Device(void);
	case  	EResetDevice: pCurrentMotor->dSPIN_Reset_Device();

						//while(pCurrentMotor->dSPIN_Busy_HW());
						break;

						//void dSPIN_Soft_Stop(void);
	case  	ESoftStop:	pCurrentMotor->dSPIN_Soft_Stop();
						break;

						//void dSPIN_Hard_Stop(void);
	case  	EHardStop:	pCurrentMotor->dSPIN_Hard_Stop();
						break;

						//void dSPIN_Soft_HiZ(void);
	case  	ESoftHIZ:	pCurrentMotor->dSPIN_Soft_HiZ();
						adcdata[12] = Motor1.dSPIN_Get_ElePos();
						break;

						//void dSPIN_Hard_HiZ(void);
	case  	EHardHiZ:	pCurrentMotor->dSPIN_Hard_HiZ();
						adcdata[12] = Motor1.dSPIN_Get_ElePos();
						break;

						//uint16_t dSPIN_Get_Status(void);
	case  	EGetStatus:	pCurrentMotor->dSPIN_Get_Status();
						break;

	}


}

void *ADCThread(void * ptr)
{
//	int len;

	int pdata;
	static __u8 ch = 0;
	static int convcnt = 0;

	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);

	while(1)
	{
		if(MotorCmdExecuting !=2)
		{
			//BBBGpio.SetSpiCs(AdcCsn);

		Max1300adc.SetConfiguration(ch,0);
		//Max1300adc.SetChRange(ch,);
		BBBGpio.gpio_set_value(AdcCsngpio, LOW);
		Max1300adc.WriteConfiguration(ch);
		BBBGpio.gpio_set_value(AdcCsngpio, HIGH);

		//GetAdcChannelData(ch);

		convcnt++;
		if(convcnt == 1)
		{
			convcnt = 0;
			GetAdcChannelData(ch);

			//printf(" ch%d = %5d ",ch,adcdata[ch]);

			ch++;
			if(ch >7)
			{
				ch = 0;
				//printf("\n");
			}
		}

		else
		{
			//BBBGpio.SetSpiCs(AdcCsn);
			BBBGpio.gpio_set_value(AdcCsngpio, LOW);
			Max1300adc.Mode = 0;
			Max1300adc.GetAdcChData(ch);
			BBBGpio.gpio_set_value(AdcCsngpio, HIGH);
		}








		//make sure the right motor is selected

		//printf("MotorRegSel = %d\n",MotorRegSel);
		switch(MotorRegSel)
			{
			case 1: pCurrentMotor = pMotor1;
					break;

			case 2: pCurrentMotor = pMotor2;
			//printf("motor2 selected\n");
					break;

			case 3: pCurrentMotor = pMotor3;
					break;

			case 4: pCurrentMotor = pMotor4;
					break;

			default : pCurrentMotor = pMotor2;
					  break;

			}


		adcdata[10] = pCurrentMotor->dSPIN_Get_Status();

		adcdata[11] = pCurrentMotor->dSPIN_Get_Param(dSPIN_ABS_POS);
		if((adcdata[11] & 0x200000)==0x200000)adcdata[11] |=0xffc00000;
		//printf("\ndSPIN_ABS_POS=%d",adcdata[11]);
		//adcdata[12] = Motor1.dSPIN_Get_Param(dSPIN_EL_POS);
		//printf("\ndSPIN_EL_POS=%d",adcdata[12]);
		adcdata[13] = pCurrentMotor->dSPIN_Get_Param(dSPIN_MARK);
		//printf("\ndSPIN_MARK=%d\n",adcdata[13]);
		adcdata[14] = pCurrentMotor->dSPIN_Get_Param(dSPIN_SPEED);
		adcdata[15] = pCurrentMotor->dSPIN_Get_Param(dSPIN_STEP_MODE);

		adcdata[16] = pCurrentMotor->dSPIN_Get_Param(dSPIN_ACC);
		adcdata[17] = pCurrentMotor->dSPIN_Get_Param(dSPIN_DEC);
		adcdata[18] = pCurrentMotor->dSPIN_Get_Param(dSPIN_MAX_SPEED);
		adcdata[19] = pCurrentMotor->dSPIN_Get_Param(dSPIN_MIN_SPEED);
		adcdata[20] = pCurrentMotor->dSPIN_Get_Param(dSPIN_FS_SPD);

		adcdata[21] = pCurrentMotor->dSPIN_Get_Param(dSPIN_ADC_OUT);
		//printf("\nadc=%d",adcdata[21]);

		//printf("\ndSPIN_SPEED=%d\n",adcdata[14]);
		if(MotorCmdExecuting == 1)
			{
			MotorCmdExecuting = 2;
			//printf("\nmotor executing command=%d\n",MotorCmdExecuting);
			}
		}

	}
	return(NULL);
}

void DisplayRxData()
{
	printf("\nPacket Type = %X",PeripheralData.PacketType);
	printf("\nPwmA = %d",PeripheralData.PwmA);
	printf("\nPwmB = %d",PeripheralData.PwmB);
	printf("\nPwmC = %d",PeripheralData.PwmC);
	printf("\nPwmD = %d",PeripheralData.PwmD);
	printf("\nPortADir = %d",PeripheralData.PortADir);
	printf("\nPortBDir = %d",PeripheralData.PortBDir);
	printf("\nPortADataIn = %d",PeripheralData.PortADataIn);
	printf("\nPortADataOut = %d",PeripheralData.PortADataOut);
	printf("\nPortBDataIn = %d",PeripheralData.PortBDataIn);
	printf("\nPortBDataOut = %d",PeripheralData.PortBDataOut);
	printf("\nAin0Range = %d",PeripheralData.Ain0Range);
	printf("\nAin1Range = %d",PeripheralData.Ain1Range);
	printf("\nAin2Range = %d",PeripheralData.Ain2Range);
	printf("\nAin3Range = %d",PeripheralData.Ain3Range);
	printf("\nAin4Range = %d",PeripheralData.Ain4Range);
	printf("\nAin5Range = %d",PeripheralData.Ain5Range);
	printf("\nAin6Range = %d",PeripheralData.Ain6Range);
	printf("\nAin7Range = %d",PeripheralData.Ain7Range);
	printf("\nAin0Data = %d",PeripheralData.Ain0Data);
	printf("\nAin1Data = %d",PeripheralData.Ain1Data);
	printf("\nAin2Data = %d",PeripheralData.Ain2Data);
	printf("\nAin3Data = %d",PeripheralData.Ain3Data);
	printf("\nAin4Data = %d",PeripheralData.Ain4Data);
	printf("\nAin5Data = %d",PeripheralData.Ain5Data);
	printf("\nAin6Data = %d",PeripheralData.Ain6Data);
	printf("\nAin7Data = %d\n\n",PeripheralData.Ain7Data);
}



void SetUpPWM()
{
	//const int delayMS = 10;

	std::cout << PWM::GetCapeManagerSlot("P8_13") << std::endl;
	std::cout << PWM::GetCapeManagerSlot("P8_19") << std::endl;
	std::cout << PWM::GetCapeManagerSlot("P9_14") << std::endl;
	std::cout << PWM::GetCapeManagerSlot("P9_16") << std::endl;

	// Enable both only after we have set the periods properly.
	// Otherwise we will have conflicts since each pin will try to set its own period and conflict with the others
	pinA.Enable();
	pinB.Enable();
	pinC.Enable();
	pinD.Enable();

	std::cout << "Pins Setup and ready to go!" << std::endl;

	pinA.SetDutyUS(1500);
	pinB.SetDutyUS(1500);
	pinC.SetDutyUS(1500);
	pinD.SetDutyUS(1500);
}



void InitHW(void)
{

	SetUpPWM();

	cout << "Setting up SPI" << endl;
	SPI0.SpiInit(0);
	SPI.SpiInit(1);

	pSpi0 = &SPI0;
	pSpi = &SPI;
	pBBBGpio = &BBBGpio;
	pMotorGPIO = &MotorGPIO;
	
	cout << "Setting up BBB GPIO Pins" << endl;
	BBBGpio.InitBBBGpio();

	//setup mcp23s17 gpio for motor control lines
	//sets up all 16 lines
	cout << "Setting up Motor GPIO Pins" << endl;
	BBBGpio.SetSpiCs(StatusCsn);
	MotorGPIO.AddressEnable(pSpi);
	MotorGPIO.pinMode(0x7bdb);
	MotorGPIO.digitalWrite(0xffff);

	//set up mcp23s17 gpio pins for digital IO
	//sets up all 16 lines
	cout << "Setting up Digital IO GPIO Pins" << endl;
	//BBBGpio.SetSpiCs(DioCsn);
	BBBGpio.gpio_set_value(DigIOCsngpio, LOW);
	DigitalGPIO.AddressEnable(pSpi0);
	BBBGpio.gpio_set_value(DigIOCsngpio, HIGH);
	BBBGpio.gpio_set_value(DigIOCsngpio, LOW);
	DigitalGPIO.pinMode(0x0);
	BBBGpio.gpio_set_value(DigIOCsngpio, HIGH);

	cout << "Initializing Max1300 ADC" << endl;
	//BBBGpio.SetSpiCs(AdcCsn);
	BBBGpio.gpio_set_value(AdcCsngpio, LOW);
	Max1300adc.InitAdc(pSpi0);
	BBBGpio.gpio_set_value(AdcCsngpio, HIGH);

	Max1300adc.Mode = 0;
	BBBGpio.gpio_set_value(AdcCsngpio, LOW);
	Max1300adc.WriteMode(0);
	BBBGpio.gpio_set_value(AdcCsngpio, HIGH);
	Max1300adc.SetConfiguration(0,0);
	Max1300adc.SetChRange(0,0);

	//init each motor controller last

	InitAllMotors();

	TestAllMotors();

}

void ProbeMotors()
{
	__u32 retval;
	MotorsInstalled = 0;

	Motor1.dSPIN_Set_Param(dSPIN_MARK, 0x1aa55);

	retval = Motor1.dSPIN_Get_Param(dSPIN_MARK);

	if(retval == 0x1aa55)
	{
		MotorsInstalled |= 1;
	}

	Motor2.dSPIN_Set_Param(dSPIN_MARK, 0x2aa55);
	retval = Motor2.dSPIN_Get_Param(dSPIN_MARK);

	if(retval == 0x2aa55)
	{
		MotorsInstalled |= 2;
	}

	Motor3.dSPIN_Set_Param(dSPIN_MARK, 0x3aa55);
	retval = Motor3.dSPIN_Get_Param(dSPIN_MARK);

	if(retval == 0x3aa55)
	{
		MotorsInstalled |= 4;
	}

	Motor4.dSPIN_Set_Param(dSPIN_MARK, 0x4aa55);
	retval = Motor4.dSPIN_Get_Param(dSPIN_MARK);

	if(retval == 0x4aa55)
	{
		MotorsInstalled |= 8;
	}


}

void InitAllMotors()
{


	Motor1.InitMotor(pSpi,Motor1Csn,pBBBGpio,pMotorGPIO);
	Motor2.InitMotor(pSpi,Motor2Csn,pBBBGpio,pMotorGPIO);
	Motor3.InitMotor(pSpi,Motor3Csn,pBBBGpio,pMotorGPIO);
	Motor4.InitMotor(pSpi,Motor4Csn,pBBBGpio,pMotorGPIO);


	Motor1.ResetL6470();
	Motor2.ResetL6470();
	Motor3.ResetL6470();
	Motor4.ResetL6470();
	while(Motor1.dSPIN_Busy_HW())
			{
				;
			}

	printf("Probing Motors\n");
	ProbeMotors();

	if((MotorsInstalled & 1) ==1)
	{
		printf("L6470-1 Detected\n");
		Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
		Motor1.SetupL6470();

	}

	if(MotorsInstalled & 2)
	{
		printf("L6470-2 Detected\n");
		Motor2.dSPIN_rx_data = Motor2.dSPIN_Get_Status();
		Motor2.SetupL6470();

	}

	if(MotorsInstalled & 4)
	{
		printf("L6470-3 Detected\n");
		Motor3.dSPIN_rx_data = Motor3.dSPIN_Get_Status();
		Motor3.SetupL6470();
	}

	if(MotorsInstalled & 8)
	{
		printf("L6470-4 Detected\n");
		Motor4.dSPIN_rx_data = Motor4.dSPIN_Get_Status();
		Motor4.SetupL6470();
	}

	//WhileMotorsBusy();






	//WhileMotorsBusy();








	//Motor1.SetupL6470();
	//Motor2.SetupL6470();
	//Motor3.SetupL6470();
	//Motor4.SetupL6470();


	pMotor1 = &Motor1;
	pMotor2 = &Motor2;
	pMotor3 = &Motor3;
	pMotor4 = &Motor4;

	//Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();

	//printf("waiting for busy\n");
	//usleep(5000000);
	//while(Motor1.dSPIN_Busy_HW());
	//Motor1.dSPIN_Hard_HiZ();
	//printf("waiting for busy1\n");
	//usleep(5000000);
	//while(Motor1.dSPIN_Busy_HW());
	//Motor1.dSPIN_Hard_Stop();
	//printf("waiting for busy2\n");
	//while(Motor1.dSPIN_Busy_HW());
	//Motor1.dSPIN_Hard_HiZ();
	//usleep(5000000);
	//while(Motor1.dSPIN_Busy_HW());
	//printf("writing dSPIN_EL_POS\n");
	//while(1)
	//{
	//Motor1.dSPIN_Hard_Stop();
	//Motor1.dSPIN_Hard_HiZ();
	//Motor1.dSPIN_Set_ElePos(0x1bb);
	//adcdata[12] = Motor1.dSPIN_Get_ElePos();
	//}
	//while(Motor1.dSPIN_Busy_HW());
	//usleep(5000000);
	//printf("reading dSPIN_EL_POS\n");
	//while(1)
	//{
	//Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
	//		printf("Motor1-1 Status=%.4X\n", Motor1.dSPIN_rx_data);
	//adcdata[12] = Motor1.dSPIN_Get_ElePos();
	//}
	//Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
	//	printf("Motor1-1 Status=%.4X\n", Motor1.dSPIN_rx_data);
	//printf("dSPIN_EL_POS=%d\n",adcdata[12]);
	//while(Motor1.dSPIN_Busy_HW());
	//usleep(5000000);
}

void WhileMotorsBusy()
{
	while(Motor1.dSPIN_Busy_HW())
		{
			;
		}
	while(Motor2.dSPIN_Busy_HW())
				{
					;
				}
	while(Motor3.dSPIN_Busy_HW())
		{
			;
		}
	while(Motor4.dSPIN_Busy_HW())
				{
					;
				}
}

void TestAllMotors()
{
	__u32 retval;
	Motor1.dSPIN_Reset_Pos();
	Motor2.dSPIN_Reset_Pos();
	Motor3.dSPIN_Reset_Pos();
	Motor4.dSPIN_Reset_Pos();
	Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
	Motor2.dSPIN_rx_data = Motor2.dSPIN_Get_Status();
	Motor3.dSPIN_rx_data = Motor3.dSPIN_Get_Status();
	Motor4.dSPIN_rx_data = Motor4.dSPIN_Get_Status();
	printf("\nMotor1-1 Status=%.4X", Motor1.dSPIN_rx_data);


	WhileMotorsBusy();
	//usleep(5000000);
	/* Move by 60,000 steps rorward, range 0 to 4,194,303 */
	printf("\nmoving forward");
	Motor1.dSPIN_Move(FWD, (uint32_t)(100000));
	Motor2.dSPIN_Move(FWD, (uint32_t)(100000));
	Motor3.dSPIN_Move(FWD, (uint32_t)(100000));
	Motor4.dSPIN_Move(FWD, (uint32_t)(100000));

	WhileMotorsBusy();

	printf("\ngoing home");

	Motor1.dSPIN_Go_Home();
	Motor2.dSPIN_Go_Home();
	Motor3.dSPIN_Go_Home();
	Motor4.dSPIN_Go_Home();

	WhileMotorsBusy();

	/* Read Status register content */
	Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
	printf("\nMotor1 Status=%.4X", Motor1.dSPIN_rx_data);

	Motor1.dSPIN_Hard_HiZ();
	Motor2.dSPIN_Hard_HiZ();
	Motor3.dSPIN_Hard_HiZ();
	Motor4.dSPIN_Hard_HiZ();

	WhileMotorsBusy();

	/* Read Status register content */
	Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
	printf("\nMotor1 Status=%.4X", Motor1.dSPIN_rx_data);
	Motor2.dSPIN_rx_data = Motor2.dSPIN_Get_Status();
	Motor3.dSPIN_rx_data = Motor3.dSPIN_Get_Status();
	Motor4.dSPIN_rx_data = Motor4.dSPIN_Get_Status();
	printf("done with testing motors\n");

	return;
	//Motor1.dSPIN_Go_Until(0,REV,5000);

	/*
	while(1)
	{
		Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
		printf("\nMotor1 Status=%.4X", Motor1.dSPIN_rx_data);

		//BBBGpio.gpio_set_value(GPIO318,LOW);
		//BBBGpio.gpio_set_value(GPIO318,HIGH);
		//printf("\n value = %d",value);
	}
*/
	//while(!Motor1.dSPIN_Busy_HW());
	//usleep(1000);

		/* Wait untill not busy - busy pin test */
	while(Motor1.dSPIN_Busy_HW())
	{
		//BBBGpio.SetSpiCs(AdcCsn);
		//retval = Max1300adc.GetAdcChData(0);
		//printf("\n%.2X ", retval);
		//Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
				//printf("\n1 Motor1 Status=%.4X", Motor1.dSPIN_rx_data);

	}
	printf("\n1 Motor1 Status=%.4X", Motor1.dSPIN_rx_data);
/*
	while(Motor2.dSPIN_Busy_HW())
	{
	    //retval = Max1300adc.GetAdcChData(0);
		//printf("\n%.2X ", retval);
	}
	while(Motor3.dSPIN_Busy_HW())
	{
	    //retval = Max1300adc.GetAdcChData(0);
		//printf("\n%.2X ", retval);
	}
	while(Motor4.dSPIN_Busy_HW())
	{
	    //retval = Max1300adc.GetAdcChData(0);
		//printf("\n%.2X ", retval);
	}
*/
	/* Send dSPIN command change hold duty cycle to 0.5% */
	////cg//dSPIN_Set_Param(dSPIN_KVAL_HOLD, Kval_Perc_to_Par(0.5));

	/* Send dSPIN command change run duty cycle to 5% */
	//cg//dSPIN_Set_Param(dSPIN_KVAL_RUN, Kval_Perc_to_Par(5));

	/* Run constant speed of 50 steps/s reverse direction */
	Motor1.dSPIN_Run(REV, Speed_Steps_to_Par(100));
	//Motor2.dSPIN_Run(REV, Speed_Steps_to_Par(500));
	//Motor3.dSPIN_Run(REV, Speed_Steps_to_Par(500));
	//Motor4.dSPIN_Run(REV, Speed_Steps_to_Par(500));

	/* Wait few seconds - motor turns */
	//Delay(0x00FFFFFF);
	Motor1.dSPIN_Go_Home();
	usleep(1000000);

	/* Perform SoftStop commmand */
	Motor1.dSPIN_Hard_Stop();
	//Motor2.dSPIN_Soft_Stop();
	//Motor3.dSPIN_Soft_Stop();
	//Motor4.dSPIN_Soft_Stop();

	/* Wait untill not busy - busy status check in Status register */
	while(Motor1.dSPIN_Busy_HW())
	{
			//BBBGpio.SetSpiCs(AdcCsn);
			//retval = Max1300adc.GetAdcChData(0);
			//printf("\n%.2X ", retval);
		Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
		printf("\n2 Motor1 Status=%.4X", Motor1.dSPIN_rx_data);

	}

	//while(Motor2.dSPIN_Busy_HW());//HW
	//while(Motor3.dSPIN_Busy_HW());//HW
	//while(Motor4.dSPIN_Busy_HW());//HW

	/* Move by 100,000 steps forward, range 0 to 4,194,303 */
	Motor1.dSPIN_Move(FWD, (uint32_t)(10000));
	//Motor2.dSPIN_Move(FWD, (uint32_t)(100000));
	//Motor3.dSPIN_Move(FWD, (uint32_t)(100000));
	//Motor4.dSPIN_Move(FWD, (uint32_t)(100000));

	/* Wait untill not busy */
	while(Motor1.dSPIN_Busy_HW())
	{
			//BBBGpio.SetSpiCs(AdcCsn);
			//retval = Max1300adc.GetAdcChData(0);
			//printf("\n%.2X ", retval);
		Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
				printf("\n3 Motor1 Status=%.4X", Motor1.dSPIN_rx_data);

		}//HW
	//while(Motor2.dSPIN_Busy_HW());//HW
	//while(Motor3.dSPIN_Busy_HW());//HW
	//while(Motor4.dSPIN_Busy_HW());//HW

	/* Test of the Flag pin by polling, wait in endless cycle if problem is detected */
	while(!Motor1.dSPIN_Flag())
	{
		Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
		printf("\n4 Motor1 Status=%.4X", Motor1.dSPIN_rx_data);

	}

	/* Issue dSPIN Go Home command */
	Motor1.dSPIN_Go_Home();
	//Motor2.dSPIN_Go_Home();
	//Motor3.dSPIN_Go_Home();
	//Motor4.dSPIN_Go_Home();

	/* Wait untill not busy - busy pin test */
	/* Wait untill not busy */
	while(Motor1.dSPIN_Busy_HW());//HW
	//while(Motor2.dSPIN_Busy_HW());//HW
	//while(Motor3.dSPIN_Busy_HW());//HW
	//while(Motor4.dSPIN_Busy_HW());//HW

	/* Issue dSPIN Go To command */
	Motor1.dSPIN_Go_To(0x0000FFFF);
	//Motor2.dSPIN_Go_To(0x0000FFFF);
	//Motor3.dSPIN_Go_To(0x0000FFFF);
	//Motor4.dSPIN_Go_To(0x0000FFFF);


	/* Wait untill not busy - busy pin test */
	/* Wait untill not busy */
	while(Motor1.dSPIN_Busy_HW());//HW
	//while(Motor2.dSPIN_Busy_HW());//HW
	//while(Motor3.dSPIN_Busy_HW());//HW
	//while(Motor4.dSPIN_Busy_HW());//HW
	Motor1.dSPIN_Go_Home();
	/* Issue dSPIN Go To command */
	Motor1.dSPIN_Go_To_Dir(FWD, 0x0001FFFF);
	//Motor2.dSPIN_Go_To_Dir(FWD, 0x0001FFFF);
	//Motor3.dSPIN_Go_To_Dir(FWD, 0x0001FFFF);
	//Motor4.dSPIN_Go_To_Dir(FWD, 0x0001FFFF);


	/* Wait untill not busy - busy pin test */
	/* Wait untill not busy */
	while(Motor1.dSPIN_Busy_HW());//HW
	//while(Motor2.dSPIN_Busy_HW());//HW
	//while(Motor3.dSPIN_Busy_HW());//HW
	//while(Motor4.dSPIN_Busy_HW());//HW
	Motor1.dSPIN_Go_Home();
	Motor1.dSPIN_Hard_HiZ();
	//Motor2.dSPIN_Hard_HiZ();
	//Motor3.dSPIN_Hard_HiZ();
	//Motor4.dSPIN_Hard_HiZ();

	/* Wait untill not busy */
	while(Motor1.dSPIN_Busy_HW());//HW
	//while(Motor2.dSPIN_Busy_HW());//HW
	//while(Motor3.dSPIN_Busy_HW());//HW
	//while(Motor4.dSPIN_Busy_HW());//HW
	Motor1.dSPIN_Go_Home();
	/* Reset position counter */
	//Motor1.dSPIN_Reset_Pos();
	//Motor2.dSPIN_Reset_Pos();
	//Motor3.dSPIN_Reset_Pos();
	//Motor4.dSPIN_Reset_Pos();

	/* Issue dSPIN Hard HiZ command - disable power stage (High Impedance) */
	Motor1.dSPIN_Hard_HiZ();
	//Motor2.dSPIN_Hard_HiZ();
	//Motor3.dSPIN_Hard_HiZ();
	//Motor4.dSPIN_Hard_HiZ();

	/* Wait untill not busy */
	while(Motor1.dSPIN_Busy_HW());//HW
	//while(Motor2.dSPIN_Busy_HW());//HW
	//while(Motor3.dSPIN_Busy_HW());//HW
	//while(Motor4.dSPIN_Busy_HW());//HW

	/* Read run duty cycle (dSPIN_KVAL_RUN) parameter from dSPIN */
	Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Param(dSPIN_KVAL_RUN);
	printf("\ndSPIN_KVAL_RUN=%.4X", Motor1.dSPIN_rx_data);

	/* Read intersect speed (dSPIN_INT_SPD) parameter from dSPIN */
	Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Param(dSPIN_INT_SPD);
	printf("\ndSPIN_INT_SPD=%.4X", Motor1.dSPIN_rx_data);

	/* Read Status register content */
	Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Status();
	printf("\nMotor1 Status=%.4X", Motor1.dSPIN_rx_data);

	/* Read absolute position (dSPIN_ABS_POS) parameter from dSPIN */
	Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Param(dSPIN_ABS_POS);
	printf("\ndSPIN_ABS_POS=%.4X", Motor1.dSPIN_rx_data);

	/* Reset position counter */
	Motor1.dSPIN_Reset_Pos();

	/* Read absolute position (dSPIN_ABS_POS) parameter from dSPIN */
	Motor1.dSPIN_rx_data = Motor1.dSPIN_Get_Param(dSPIN_ABS_POS);
	printf("\ndSPIN_ABS_POS rst=%.4X\n", Motor1.dSPIN_rx_data);
}

void GetAdcChannelData(__u8 ch)
{
	//static __u8 ch = 0;
	__u32 retval;
	//BBBGpio.SetSpiCs(AdcCsn);
	BBBGpio.gpio_set_value(AdcCsngpio, LOW);
	Max1300adc.Mode = 0;
	//Max1300adc.WriteMode(0);
	//Max1300adc.SetConfiguration(ch,0);
	//Max1300adc.SetChRange(ch,1);
	adcdata[ch]= Max1300adc.GetAdcChData(ch);
	BBBGpio.gpio_set_value(AdcCsngpio, HIGH);
	//printf("\n%.2X ", retval);
	//printf(" ch%d = %5d ",ch,adcdata[ch]);
	//ch++;

	//if(ch>7)
	//{
		//printf("\n");
	//	ch = 0;

	//}

	//Max1300adc.SetConfiguration(ch,0);
	//Max1300adc.SetChRange(ch,1);
	//Max1300adc.WriteConfiguration(ch);

	return;
}

















