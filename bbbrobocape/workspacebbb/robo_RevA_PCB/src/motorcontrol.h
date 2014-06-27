#ifndef DSPIN_H_
#define DSPIN_H_
/*
 * dspin.h
 *
 *  Created on: Sep 29, 2013
 *      Author: Chris
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


#include "BBBGPIO.h"
#include "MCP23S17.h"
#include "Spi.h"




/* dSPIN electrical position register masks */
#define dSPIN_ELPOS_STEP_MASK			((uint8_t)0xC0)
#define dSPIN_ELPOS_MICROSTEP_MASK		((uint8_t)0x3F)

/* dSPIN min speed register bit / mask */
#define dSPIN_LSPD_OPT			((uint16_t)0x1000)
#define dSPIN_MIN_SPEED_MASK	((uint16_t)0x0FFF)



/* Exported types ------------------------------------------------------------*/

/**
  * @brief dSPIN Init structure definition
  */
typedef struct
{
  uint32_t ABS_POS;
  uint16_t EL_POS;
  uint32_t MARK;
  uint32_t SPEED;
  uint16_t ACC;
  uint16_t DEC;
  uint16_t MAX_SPEED;
  uint16_t MIN_SPEED;
  uint16_t FS_SPD;
  uint8_t  KVAL_HOLD;
  uint8_t  KVAL_RUN;
  uint8_t  KVAL_ACC;
  uint8_t  KVAL_DEC;
  uint16_t INT_SPD;
  uint8_t  ST_SLP;
  uint8_t  FN_SLP_ACC;
  uint8_t  FN_SLP_DEC;
  uint8_t  K_THERM;
  uint8_t  ADC_OUT;
  uint8_t  OCD_TH;
  uint8_t  STALL_TH;
  uint8_t  STEP_MODE;
  uint8_t  ALARM_EN;
  uint16_t CONFIG;
}dSPIN_RegsStruct_TypeDef;



/* Exported macro ------------------------------------------------------------*/
#define Speed_Steps_to_Par(steps) ((uint32_t)(((steps)*67.108864)+0.5))			/* Speed conversion, range 0 to 15625 steps/s */
#define AccDec_Steps_to_Par(steps) ((uint16_t)(((steps)*0.068719476736)+0.5))	/* Acc/Dec rates conversion, range 14.55 to 59590 steps/s2 */
#define MaxSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*0.065536)+0.5))			/* Max Speed conversion, range 15.25 to 15610 steps/s */
#define MinSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*4.194304)+0.5))			/* Min Speed conversion, range 0 to 976.3 steps/s */
#define FSSpd_Steps_to_Par(steps) ((uint16_t)((steps)*0.065536))				/* Full Step Speed conversion, range 7.63 to 15625 steps/s */
#define IntSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*4.194304)+0.5))			/* Intersect Speed conversion, range 0 to 3906 steps/s */
#define Kval_Perc_to_Par(perc) ((uint8_t)(((perc)/0.390625)+0.5))				/* KVAL conversions, range 0.4% to 99.6% */
#define BEMF_Slope_Perc_to_Par(perc) ((uint8_t)(((perc)/0.00156862745098)+0.5))	/* BEMF compensation slopes, range 0 to 0.4% s/step */
#define KTherm_to_Par(KTherm) ((uint8_t)(((KTherm - 1)/0.03125)+0.5))			/* K_THERM compensation conversion, range 1 to 1.46875 */
#define StallTh_to_Par(StallTh) ((uint8_t)(((StallTh - 31.25)/31.25)+0.5))		/* Stall Threshold conversion, range 31.25mA to 4000mA */

//dSPIN_RegsStruct_TypeDef dSPIN_RegsStruct;



class L6470Motor
{
public:


	SPIClass *pSPI;
	uint8_t src_addr[32];
	uint8_t dest_addr[32];
	uint16_t tempstatus;
	uint32_t dSPIN_rx_data;
	__u8 SpiCsn;
	BBBGPIO *pBBBGpio;
	MCP *pMotorGPIO;

	dSPIN_RegsStruct_TypeDef dSPIN_RegsStruct;

	void TestMotor(void);
	//void InitMotor(SPIClass *SPIp,__u8 spicsn,BBBGPIO *BBBGpiop,MCP *MotorGPIOp);
	void InitMotor(SPIClass *SPIp,__u8 spicsn,BBBGPIO *BBBGpiop,MCP *MotorGPIOp);
	void Delay(uint32_t nCount);
	void dSPIN_Peripherals_Init(void);
	void dSPIN_Regs_Struct_Reset(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct);
	void dSPIN_Registers_Set(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct);
	void dSPIN_Nop(void);
	void dSPIN_Set_Param(__u8 param, __u32 value);
	uint32_t dSPIN_Get_Param(__u8 param);
	void dSPIN_Run(uint8_t direction, uint32_t speed);
	void dSPIN_Step_Clock(uint8_t direction);
	void dSPIN_Move(uint8_t direction, uint32_t n_step);
	void dSPIN_Go_To(uint32_t abs_pos);
	void dSPIN_Go_To_Dir(uint8_t direction, uint32_t abs_pos);
	void dSPIN_Go_Until(uint8_t action, uint8_t direction, uint32_t speed);
	void dSPIN_Release_SW(uint8_t action, uint8_t direction);
	void dSPIN_Go_Home(void);
	void dSPIN_Go_Mark(void);
	void dSPIN_Reset_Pos(void);
	void dSPIN_Reset_Device(void);
	void dSPIN_Soft_Stop(void);
	void dSPIN_Hard_Stop(void);
	void dSPIN_Soft_HiZ(void);
	void dSPIN_Hard_HiZ(void);
	uint16_t dSPIN_Get_Status(void);
	uint8_t dSPIN_Busy_HW(void);
	uint8_t dSPIN_Busy_SW(void);
	uint8_t dSPIN_Flag(void);
	int SetupL6470(void);
	uint8_t dSPIN_Write_Byte(uint8_t byte);
	void ResetL6470(void);
	void dSPIN_Set_ElePos(int value);
	int dSPIN_Get_ElePos(void);
};

#endif /* DSPIN_H_ */
