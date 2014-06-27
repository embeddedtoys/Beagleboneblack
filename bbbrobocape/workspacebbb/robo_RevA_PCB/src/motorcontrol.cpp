#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "motorcontrol.h"
//include for setting spi motor select and status read
//#include "BBBGPIO.h"
//#include "MCP23S17.h"
//include for spi procedures
//#include "Spi.h"

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


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

/******************************************************************************
**   InitMotors()
******************************************************************************/

void L6470Motor::InitMotor(SPIClass *SPIp,__u8 spicsn,BBBGPIO *BBBGpiop,MCP *MotorGPIOp)
{
	pSPI = SPIp;
	SpiCsn = spicsn;
	pBBBGpio = BBBGpiop;
	pMotorGPIO=MotorGPIOp;
	dSPIN_Reset_Device();
	SetupL6470();
}


int L6470Motor::SetupL6470(void)
{

	/* Structure initialization by default values, in order to avoid blank records */
	dSPIN_Regs_Struct_Reset(&dSPIN_RegsStruct);

	/* Acceleration rate settings to 466 steps/s2, range 14.55 to 59590 steps/s2 */
	dSPIN_RegsStruct.ACC 		= AccDec_Steps_to_Par(500);
	/* Deceleration rate settings to 466 steps/s2, range 14.55 to 59590 steps/s2 */
	dSPIN_RegsStruct.DEC 		= AccDec_Steps_to_Par(500);
	/* Maximum speed settings to 488 steps/s, range 15.25 to 15610 steps/s */
	dSPIN_RegsStruct.MAX_SPEED 	= MaxSpd_Steps_to_Par(500);
	/* Minimum speed settings to 0 steps/s, range 0 to 976.3 steps/s */
	dSPIN_RegsStruct.MIN_SPEED	= MinSpd_Steps_to_Par(0);
	/* Full step speed settings 252 steps/s, range 7.63 to 15625 steps/s */
	dSPIN_RegsStruct.FS_SPD 	= FSSpd_Steps_to_Par(800);
	/* Hold duty cycle (torque) settings to 10%, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_HOLD 	= Kval_Perc_to_Par(10);
	/* Run duty cycle (torque) settings to 10%, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_RUN 	= Kval_Perc_to_Par(35);
	/* Acceleration duty cycle (torque) settings to 10%, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_ACC 	= Kval_Perc_to_Par(35);
	/* Deceleration duty cycle (torque) settings to 10%, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_DEC 	= Kval_Perc_to_Par(35);
	/* Intersect speed settings for BEMF compensation to 200 steps/s, range 0 to 3906 steps/s */
	dSPIN_RegsStruct.INT_SPD 	= IntSpd_Steps_to_Par(200);
	/* BEMF start slope settings for BEMF compensation to 0.038% step/s, range 0 to 0.4% s/step */
	dSPIN_RegsStruct.ST_SLP 	= BEMF_Slope_Perc_to_Par(0.038);
	/* BEMF final acc slope settings for BEMF compensation to 0.063% step/s, range 0 to 0.4% s/step */
	dSPIN_RegsStruct.FN_SLP_ACC = BEMF_Slope_Perc_to_Par(0.063);
	/* BEMF final dec slope settings for BEMF compensation to 0.063% step/s, range 0 to 0.4% s/step */
	dSPIN_RegsStruct.FN_SLP_DEC = BEMF_Slope_Perc_to_Par(0.063);
	/* Thermal compensation param settings to 1, range 1 to 1.46875 */
	dSPIN_RegsStruct.K_THERM 	= KTherm_to_Par(1);
	/* Overcurrent threshold settings to 1500mA */
	dSPIN_RegsStruct.OCD_TH 	= dSPIN_OCD_TH_4125mA;
	/* Stall threshold settings to 1000mA, range 31.25 to 4000mA */
	dSPIN_RegsStruct.STALL_TH 	= StallTh_to_Par(4000);
	/* Step mode settings to 128 microsteps */
	dSPIN_RegsStruct.STEP_MODE 	= dSPIN_STEP_SEL_1_128;//dSPIN_STEP_SEL_1_2
	/* Alarm settings - all alarms enabled */
	dSPIN_RegsStruct.ALARM_EN 	= dSPIN_ALARM_EN_OVERCURRENT | dSPIN_ALARM_EN_THERMAL_SHUTDOWN
		| dSPIN_ALARM_EN_THERMAL_WARNING | dSPIN_ALARM_EN_UNDER_VOLTAGE | dSPIN_ALARM_EN_STALL_DET_A
		| dSPIN_ALARM_EN_STALL_DET_B | dSPIN_ALARM_EN_SW_TURN_ON | dSPIN_ALARM_EN_WRONG_NPERF_CMD;
	/* Internal oscillator, 2MHz OSCOUT clock, supply voltage compensation disabled, *
	 * overcurrent shutdown enabled, slew-rate = 290 V/us, PWM frequency = 15.6kHz   */
	dSPIN_RegsStruct.CONFIG 	= dSPIN_CONFIG_EXT_16MHZ_XTAL_DRIVE | dSPIN_CONFIG_SW_HARD_STOP
		| dSPIN_CONFIG_VS_COMP_DISABLE | dSPIN_CONFIG_OC_SD_ENABLE | dSPIN_CONFIG_SR_290V_us
		| dSPIN_CONFIG_PWM_DIV_2 | dSPIN_CONFIG_PWM_MUL_1;

	/* Program all dSPIN registers */
	dSPIN_Registers_Set(&dSPIN_RegsStruct);


	return 0;
}

void L6470Motor::TestMotor()
{
/* Move by 60,000 steps rorward, range 0 to 4,194,303 */
	dSPIN_Move(FWD, (uint32_t)(600000));
	//Delay(0x00F);
	/* Wait untill not busy - busy pin test */
	while(dSPIN_Busy_HW());//HW

	/* Send dSPIN command change hold duty cycle to 0.5% */
	////cg//dSPIN_Set_Param(dSPIN_KVAL_HOLD, Kval_Perc_to_Par(0.5));

	/* Send dSPIN command change run duty cycle to 5% */
	//cg//dSPIN_Set_Param(dSPIN_KVAL_RUN, Kval_Perc_to_Par(5));

	/* Run constant speed of 50 steps/s reverse direction */
	dSPIN_Run(REV, Speed_Steps_to_Par(500));

	/* Wait few seconds - motor turns */
	//Delay(0x00FFFFFF);

	usleep(5000000);

	/* Perform SoftStop commmand */
	dSPIN_Hard_Stop();

	/* Wait untill not busy - busy status check in Status register */
	while(dSPIN_Busy_HW());

	/* Move by 100,000 steps forward, range 0 to 4,194,303 */
	dSPIN_Move(FWD, (uint32_t)(100000));

	/* Wait untill not busy */
	while(dSPIN_Busy_HW());

	/* Test of the Flag pin by polling, wait in endless cycle if problem is detected */
	if(dSPIN_Flag()) while(1);

	/* Issue dSPIN Go Home command */
	dSPIN_Go_Home();
	/* Wait untill not busy - busy pin test */
	while(dSPIN_Busy_HW());

	/* Issue dSPIN Go To command */
	dSPIN_Go_To(0x0000FFFF);
	/* Wait untill not busy - busy pin test */
	while(dSPIN_Busy_HW());

	/* Issue dSPIN Go To command */
	dSPIN_Go_To_Dir(FWD, 0x0001FFFF);
	/* Wait untill not busy - busy pin test */
	while(dSPIN_Busy_HW());
	dSPIN_Hard_HiZ();


	/* Read run duty cycle (dSPIN_KVAL_RUN) parameter from dSPIN */
	dSPIN_rx_data = dSPIN_Get_Param(dSPIN_KVAL_RUN);

	/* Read intersect speed (dSPIN_INT_SPD) parameter from dSPIN */

	dSPIN_rx_data = dSPIN_Get_Param(dSPIN_INT_SPD);

	/* Read Status register content */
	dSPIN_rx_data = dSPIN_Get_Status();

	/* Read absolute position (dSPIN_ABS_POS) parameter from dSPIN */
	dSPIN_rx_data = dSPIN_Get_Param(dSPIN_ABS_POS);

	/* Reset position counter */
	dSPIN_Reset_Pos();

	/* Read absolute position (dSPIN_ABS_POS) parameter from dSPIN */
	dSPIN_rx_data = dSPIN_Get_Param(dSPIN_ABS_POS);

	/* Issue dSPIN Hard HiZ command - disable power stage (High Impedance) */
	dSPIN_Hard_HiZ();
return;
}






/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
void L6470Motor::Delay(uint32_t nCount)
{
  for(; nCount!= 0;nCount--);

}



/**
  * @brief  Fills-in dSPIN configuration structure with default values.
  * @param  Structure address (pointer to struct)
  * @retval None
  */
void L6470Motor::dSPIN_Regs_Struct_Reset(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct)
{
	dSPIN_RegsStruct->ABS_POS = 0;
	dSPIN_RegsStruct->EL_POS = 0;
	dSPIN_RegsStruct->MARK = 0;
	dSPIN_RegsStruct->SPEED = 0;
	//dSPIN_RegsStruct->ACC = 0x08A;
	//dSPIN_RegsStruct->DEC = 0x08A;
	dSPIN_RegsStruct->ACC = 0xfff;
	dSPIN_RegsStruct->DEC = 0xfff;
	dSPIN_RegsStruct->MAX_SPEED = 0x041;
	dSPIN_RegsStruct->MIN_SPEED = 0;
	dSPIN_RegsStruct->FS_SPD = 0x027;
	dSPIN_RegsStruct->KVAL_HOLD = 0x29;
	dSPIN_RegsStruct->KVAL_RUN = 0x29;
	dSPIN_RegsStruct->KVAL_ACC = 0x29;
	dSPIN_RegsStruct->KVAL_DEC = 0x29;
	dSPIN_RegsStruct->INT_SPD = 0x0408;
	dSPIN_RegsStruct->ST_SLP = 0x19;
	dSPIN_RegsStruct->FN_SLP_ACC = 0x29;
	dSPIN_RegsStruct->FN_SLP_DEC = 0x29;
	dSPIN_RegsStruct->K_THERM = 0;
	dSPIN_RegsStruct->OCD_TH = 0x8;
	dSPIN_RegsStruct->STALL_TH = 0x40;
	dSPIN_RegsStruct->STEP_MODE = 0x7;
	dSPIN_RegsStruct->ALARM_EN = 0xFF;
	dSPIN_RegsStruct->CONFIG = 0x2E8b;
}

/**
  * @brief  Configures dSPIN internal registers with values in the config structure.
  * @param  Configuration structure address (pointer to configuration structure)
  * @retval None
  */
void L6470Motor::dSPIN_Registers_Set(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct)
{

	dSPIN_Set_Param(dSPIN_ABS_POS, dSPIN_RegsStruct->ABS_POS);
	dSPIN_Set_Param(dSPIN_EL_POS, dSPIN_RegsStruct->EL_POS);
	dSPIN_Set_Param(dSPIN_MARK, dSPIN_RegsStruct->MARK);
	dSPIN_Set_Param(dSPIN_SPEED, dSPIN_RegsStruct->SPEED);
	dSPIN_Set_Param(dSPIN_ACC, dSPIN_RegsStruct->ACC);
	dSPIN_Set_Param(dSPIN_DEC, dSPIN_RegsStruct->DEC);
	dSPIN_Set_Param(dSPIN_MAX_SPEED, dSPIN_RegsStruct->MAX_SPEED);
	dSPIN_Set_Param(dSPIN_MIN_SPEED, dSPIN_RegsStruct->MIN_SPEED);
	dSPIN_Set_Param(dSPIN_FS_SPD, dSPIN_RegsStruct->FS_SPD);
	dSPIN_Set_Param(dSPIN_KVAL_HOLD, dSPIN_RegsStruct->KVAL_HOLD);
	dSPIN_Set_Param(dSPIN_KVAL_RUN, dSPIN_RegsStruct->KVAL_RUN);
	dSPIN_Set_Param(dSPIN_KVAL_ACC, dSPIN_RegsStruct->KVAL_ACC);
	dSPIN_Set_Param(dSPIN_KVAL_DEC, dSPIN_RegsStruct->KVAL_DEC);
	dSPIN_Set_Param(dSPIN_INT_SPD, dSPIN_RegsStruct->INT_SPD);
	dSPIN_Set_Param(dSPIN_ST_SLP, dSPIN_RegsStruct->ST_SLP);
	dSPIN_Set_Param(dSPIN_FN_SLP_ACC, dSPIN_RegsStruct->FN_SLP_ACC);
	dSPIN_Set_Param(dSPIN_FN_SLP_DEC, dSPIN_RegsStruct->FN_SLP_DEC);
	dSPIN_Set_Param(dSPIN_K_THERM, dSPIN_RegsStruct->K_THERM);
	dSPIN_Set_Param(dSPIN_OCD_TH, dSPIN_RegsStruct->OCD_TH);
	dSPIN_Set_Param(dSPIN_STALL_TH, dSPIN_RegsStruct->STALL_TH);
	dSPIN_Set_Param(dSPIN_STEP_MODE, dSPIN_RegsStruct->STEP_MODE);
	dSPIN_Set_Param(dSPIN_ALARM_EN, dSPIN_RegsStruct->ALARM_EN);
	dSPIN_Set_Param(dSPIN_CONFIG, dSPIN_RegsStruct->CONFIG);
}

/**
  * @brief  Issues dSPIN NOP command.
  * @param  None
  * @retval None
  */
void L6470Motor::dSPIN_Nop(void)
{
	/* Send NOP operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_NOP);
}

/**
  * @brief  Issues dSPIN Set Param command.
  * @param  dSPIN register address, value to be set
  * @retval None
  */
void L6470Motor::dSPIN_Set_Param(__u8 param, __u32 value)
{
	/* Send SetParam operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_SET_PARAM | param);
	switch (param)
	{
		case dSPIN_ABS_POS:
		case dSPIN_MARK:
		case dSPIN_SPEED:
			/* Send parameter - byte 2 to dSPIN */
			dSPIN_Write_Byte((uint8_t)(value >> 16));

		case dSPIN_ACC:
		case dSPIN_DEC:
		case dSPIN_MAX_SPEED:
		case dSPIN_MIN_SPEED:
		case dSPIN_FS_SPD:
		case dSPIN_INT_SPD:
		case dSPIN_CONFIG:
		case dSPIN_STATUS:

			/* Send parameter - byte 1 to dSPIN */
		   	dSPIN_Write_Byte((uint8_t)(value >> 8));
		default:
			/* Send parameter - byte 0 to dSPIN */
		   	dSPIN_Write_Byte((uint8_t)(value));
	}
}


void L6470Motor::dSPIN_Set_ElePos(int value)
{
	/* Send SetParam operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_SET_PARAM | 2);

	dSPIN_Write_Byte((uint8_t)(value >> 8));

	dSPIN_Write_Byte((uint8_t)(value & 0xff));
}

int L6470Motor::dSPIN_Get_ElePos()
{
	volatile int temp = 0;

	volatile int rx = 0;

	/* Send SetParam operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_GET_PARAM | 2);

	temp =	dSPIN_Write_Byte((uint8_t)(0x00));
	//printf("temp1 = %X\n",temp);
	temp = temp << 8;
	rx |= temp;


	temp = dSPIN_Write_Byte((uint8_t)(0x00));
	//printf("temp1 = %X\n",temp);
	rx |= temp;

	return rx;
}



/**
  * @brief  Issues dSPIN Get Param command.
  * @param  dSPIN register address
  * @retval Register value - 1 to 3 bytes (depends on register)
  */
uint32_t L6470Motor::dSPIN_Get_Param(__u8 param)
{
	volatile uint32_t temp = 0;

	volatile uint32_t rx = 0;

	rx=0;
	/* Send GetParam operation code to dSPIN */
	temp = dSPIN_Write_Byte(dSPIN_GET_PARAM | param);
	//printf("\n temp1 = %X ",temp);
	/* MSB which should be 0 */
	temp = temp << 24;
	rx |= temp;
	switch (param)
	{
		case dSPIN_ABS_POS:
		case dSPIN_MARK:
		case dSPIN_SPEED:
		   	temp = dSPIN_Write_Byte((uint8_t)(0x00));
		   	//printf(" temp2 = %X ",temp);
			temp = temp << 16;
			rx |= temp;

		case dSPIN_ACC:
		case dSPIN_DEC:
		case dSPIN_MAX_SPEED:
		case dSPIN_MIN_SPEED:
		case dSPIN_FS_SPD:
		case dSPIN_INT_SPD:
		case dSPIN_CONFIG:
		case dSPIN_STATUS:
		   	temp = dSPIN_Write_Byte((uint8_t)(0x00));
			//printf(" temp3 = %X ",temp);
			temp = temp << 8;
			rx |= temp;
		default:
		   	temp = dSPIN_Write_Byte((uint8_t)(0x00));
			//printf(" temp4 = %X \n",temp);
			rx |= temp;
	}
	return rx;
}

/**
  * @brief  Issues dSPIN Run command.
  * @param  Movement direction (FWD, REV), Speed - 3 bytes
  * @retval None
  */
void L6470Motor::dSPIN_Run(uint8_t direction, uint32_t speed)
{
	/* Send RUN operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_RUN | direction);
	/* Send speed - byte 2 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed >> 16));
	/* Send speed - byte 1 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed >> 8));
	/* Send speed - byte 0 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed));
}

/**
  * @brief  Issues dSPIN Step Clock command.
  * @param  Movement direction (FWD, REV)
  * @retval None
  */
void L6470Motor::dSPIN_Step_Clock(uint8_t direction)
{

	//GPIOSetValue( PORT2, 3, direction&1 );
	//dSPIN_Write_Byte(dSPIN_STEP_CLOCK | direction);
	//GPIOSetValue( PORT2, 4, 1 );
	Delay(1000);
	//GPIOSetValue( PORT2, 4, 0 );
}

/**
  * @brief  Issues dSPIN Move command.
  * @param  Movement direction, Number of steps
  * @retval None
  */
void L6470Motor::dSPIN_Move(uint8_t direction, uint32_t n_step)
{
	/* Send Move operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_MOVE | direction);
	/* Send n_step - byte 2 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(n_step >> 16));
	/* Send n_step - byte 1 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(n_step >> 8));
	/* Send n_step - byte 0 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(n_step));
}

/**
  * @brief  Issues dSPIN Go To command.
  * @param  Absolute position where requested to move
  * @retval None
  */
void L6470Motor::dSPIN_Go_To(uint32_t abs_pos)
{
	/* Send GoTo operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_GO_TO);
	/* Send absolute position parameter - byte 2 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos >> 16));
	/* Send absolute position parameter - byte 1 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos >> 8));
	/* Send absolute position parameter - byte 0 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos));
}

/**
  * @brief  Issues dSPIN Go To Dir command.
  * @param  Movement direction, Absolute position where requested to move
  * @retval None
  */
void L6470Motor::dSPIN_Go_To_Dir(uint8_t direction, uint32_t abs_pos)
{
	/* Send GoTo_DIR operation code to dSPIN *///ConfigData
	dSPIN_Write_Byte(dSPIN_GO_TO_DIR | direction);
	/* Send absolute position parameter - byte 2 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos >> 16));
	/* Send absolute position parameter - byte 1 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos >> 8));
	/* Send absolute position parameter - byte 0 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos));
}

/**
  * @brief  Issues dSPIN Go Until command.
  * @param  Action, Movement direction, Speed
  * @retval None
  */
void L6470Motor::dSPIN_Go_Until(uint8_t action, uint8_t direction, uint32_t speed)
{
	/* Send GoUntil operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_GO_UNTIL | action | direction);
	/* Send speed parameter - byte 2 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed >> 16));
	/* Send speed parameter - byte 1 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed >> 8));
	/* Send speed parameter - byte 0 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed));
}

/**
  * @brief  Issues dSPIN Release SW command.
  * @param  Action, Movement direction
  * @retval None
  */
void L6470Motor::dSPIN_Release_SW(uint8_t action, uint8_t direction)
{
	/* Send ReleaseSW operation code to dSPIN */
	//not implemented for package type s0 power 36
}

/**
  * @brief  Issues dSPIN Go Home command. (Shorted path to zero position)
  * @param  None
  * @retval None
  */
void L6470Motor::dSPIN_Go_Home(void)
{
	/* Send GoHome operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_GO_HOME);
}

/**
  * @brief  Issues dSPIN Go Mark command.
  * @param  None
  * @retval None
  */
void L6470Motor::dSPIN_Go_Mark(void)
{
	/* Send GoMark operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_GO_MARK);
}

/**
  * @brief  Issues dSPIN Reset Pos command.
  * @param  None
  * @retval None
  */
void L6470Motor::dSPIN_Reset_Pos(void)
{
	/* Send ResetPos operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_RESET_POS);
}

/**
  * @brief  Issues dSPIN Reset Device command.
  * @param  None
  * @retval None
  */
void L6470Motor::dSPIN_Reset_Device(void)
{
	/* Send ResetDevice operation code to dSPIN */
	//dSPIN_Write_Byte(dSPIN_RESET_DEVICE);
	//GPIOSetValue( PORT2, 1, 0 );
	//
	//GPIOSetValue( PORT2, 1, 1 );

	return
	pBBBGpio->SetSpiCs(StatusCsn);
	pMotorGPIO->digitalWrite(0xffdb);
	Delay(200);
	pMotorGPIO->digitalWrite(0xffff);
	pBBBGpio->SetSpiCs(SpiCsn);
	Delay(200);

}

/**
  * @brief  Issues dSPIN Soft Stop command.
  * @param  None
  * @retval None
  */
void L6470Motor::dSPIN_Soft_Stop(void)
{
	/* Send SoftStop operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_SOFT_STOP);
}

/**
  * @brief  Issues dSPIN Hard Stop command.
  * @param  None
  * @retval None
  */
void L6470Motor::dSPIN_Hard_Stop(void)
{
	/* Send HardStop operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_HARD_STOP);
}

/**
  * @brief  Issues dSPIN Soft HiZ command.
  * @param  None
  * @retval None
  */
void L6470Motor::dSPIN_Soft_HiZ(void)
{
	/* Send SoftHiZ operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_SOFT_HIZ);
}

/**
  * @brief  Issues dSPIN Hard HiZ command.
  * @param  None
  * @retval None
  */
void L6470Motor::dSPIN_Hard_HiZ(void)
{
	/* Send HardHiZ operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_HARD_HIZ);
}

/**
  * @brief  Issues dSPIN Get Status command.
  * @param  None
  * @retval Status Register content
  */
uint16_t L6470Motor::dSPIN_Get_Status(void)
{
	uint16_t temp = 0;
	uint16_t rx = 0;

	/* Send GetStatus operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_GET_STATUS);
	/* Send zero byte / receive MSByte from dSPIN */
	temp = dSPIN_Write_Byte((uint8_t)(0x00));
	temp = temp << 8;
	rx |= temp;
	/* Send zero byte / receive LSByte from dSPIN */
	temp = dSPIN_Write_Byte(0x00);
	rx |= temp;
	return rx;
}

/**
  * @brief  Checks if the dSPIN is Busy by hardware - active Busy signal.
  * @param  None
  * @retval one if chip is busy, otherwise zero
  */
uint8_t L6470Motor::dSPIN_Busy_HW(void)
{
	__u16 mstatus;
	uint8_t retval = 1;

	pBBBGpio->SetSpiCs(StatusCsn);
	mstatus = pMotorGPIO->digitalRead();
	//printf("\n%.2X ", mstatus);
	pBBBGpio->SetSpiCs(SpiCsn);

	switch(SpiCsn)
	{
	case Motor1Csn: if((mstatus & 0x80)==0x80) retval = 0;
					//printf("\n%d ", retval);
					break;

	case Motor2Csn: if((mstatus & 0x8)) retval = 0;
					break;

	case Motor3Csn: if((mstatus & 0x1000)) retval = 0;
					break;

	case Motor4Csn: if((mstatus & 0x4000)) retval = 0;
					break;

	default:        retval = 0;
					break;
	}

	return retval;
}

/**
  * @brief  Checks if the dSPIN is Busy by SPI - Busy flag bit in Status Register.
  * @param  None
  * @retval one if chip is busy, otherwise zero
  */
uint8_t L6470Motor::dSPIN_Busy_SW(void)
{
	if(!(dSPIN_Get_Status() & dSPIN_STATUS_BUSY)) return 0x01;
	else return 0x00;
}

/**
  * @brief  Checks dSPIN Flag signal.
  * @param  None
  * @retval one if Flag signal is active, otherwise zero
  */
uint8_t L6470Motor::dSPIN_Flag(void)
{
	__u16 mstatus;
	uint8_t retval = 0;

	pBBBGpio->SetSpiCs(StatusCsn);
	mstatus = pMotorGPIO->digitalRead();

	pBBBGpio->SetSpiCs(SpiCsn);

	switch(SpiCsn)
	{
	case Motor1Csn: if((mstatus & 0x4000)==0) retval = 1;
					break;

	case Motor2Csn: if((mstatus & 0x1000)==0) retval = 1;
					break;

	case Motor3Csn: if((mstatus & 0x8)==0) retval = 1;
					break;

	case Motor4Csn: if((mstatus & 0x20)==0) retval = 1;
					break;

	default:        retval = 0;
					break;

	}

	return retval;
}

/**
  * @brief  Transmits/Receives one byte to/from dSPIN over SPI.
  * @param  Transmited byte
  * @retval Received byte
  */
uint8_t L6470Motor::dSPIN_Write_Byte(uint8_t byte)
{
	volatile uint8_t retval;

	src_addr[0] = byte;

	pBBBGpio->SetSpiCs(SpiCsn);

	pSPI->SpiSetClkMode(SPI_MODE_3);

	retval = pSPI->SpiSendByte(byte);

	pSPI->SpiSetClkMode(SPI_MODE_0);

	pBBBGpio->SetSpiCs(0);

	return  retval;

}

void L6470Motor::ResetL6470()
{
	pBBBGpio->SetSpiCs(StatusCsn);

	switch(SpiCsn)
	{
	case Motor1Csn: pMotorGPIO->digitalWrite(0xffdf);
					Delay(200);
					pMotorGPIO->digitalWrite(0xffff);
					break;

	case Motor2Csn: pMotorGPIO->digitalWrite(0xfffb);
					Delay(200);
					pMotorGPIO->digitalWrite(0xffff);
					break;

	case Motor3Csn: pMotorGPIO->digitalWrite(0xfbff);
					Delay(200);
					pMotorGPIO->digitalWrite(0xffff);
					break;

	case Motor4Csn: pMotorGPIO->digitalWrite(0x7fff);
					Delay(200);
					pMotorGPIO->digitalWrite(0xffff);
					break;

	default:
					break;
	}
	pBBBGpio->SetSpiCs(0);
}

/******************************************************************************
**                            End Of File
******************************************************************************/
