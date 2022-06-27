/*
 * AS5047D.h
 *
 *  Created on: 24. avg. 2016
 *      Author: user
 */

#ifndef AS5047D_H_
#define AS5047D_H_

#include <stdint.h>
//#include "stm32f4xx_hal_gpio.h"
//#include "stm32f4xx_hal_gpio.h"
#include "stm32f1xx_hal.h"




// AS5047D Register Addresses
 #define EF 0x4000
/** volatile **/
#define AS5047D_NOP 0x0000
#define AS5047D_ERRFL 0x0001
#define AS5047D_PROG 0x0003
#define AS5047D_DIAAGC 0x3FFC
#define AS5047D_CORDICMAG 0x3FFD
#define AS5047D_ANGLEUNC 0x3FFE
#define AS5047D_ANGLECOM 0x3FFF

/** non-volatile **/
#define AS5047D_ZPOSM 0x0016
#define AS5047D_ZPOSL 0x0017
#define AS5047D_SETTINGS1 0x0018
#define AS5047D_SETTINGS2 0x0019

#define AS5047D_RD 0x4000    // bit 14 = "1" is Read + parity even
#define AS5047D_WR 0x3FFF    // bit 14 = "0" is Write


typedef struct
{
	SPI_HandleTypeDef*hspi_Handler;
	uint16_t nssPin;
	GPIO_TypeDef* nssPort;
}AS5047D_Handler;

void AS5047D_Init(AS5047D_Handler *AS5047D_Handler_);

HAL_StatusTypeDef AS5047D_Write(AS5047D_Handler *AS5047D_Handler_, uint16_t address, uint16_t data);
HAL_StatusTypeDef AS5047D_Write_ReadPre(AS5047D_Handler *AS5047D_Handler_, uint16_t address, uint16_t data,uint16_t* preData);
HAL_StatusTypeDef AS5047D_Read(AS5047D_Handler *AS5047D_Handler_, uint16_t address,uint16_t* pdata);

extern void AS5047D_Check_Transmission_Error(AS5047D_Handler *AS5047D_Handler_);
extern void AS5047D_SetZero(AS5047D_Handler *AS5047D_Handler_);
extern uint16_t AS5047D_GetZero(AS5047D_Handler *AS5047D_Handler_);
extern uint8_t AS5047D_Get_AGC_Value(AS5047D_Handler *AS5047D_Handler_);

extern uint16_t AS5047D_Get_CORDICMAG_Value(AS5047D_Handler *AS5047D_Handler_);
extern uint16_t AS5047D_Get_ANGLEUNC_Value(AS5047D_Handler *AS5047D_Handler_);
extern uint16_t AS5047D_Get_ANGLECOM_Value(AS5047D_Handler *AS5047D_Handler_);

extern float AS5047D_Get_True_Angle_Value(AS5047D_Handler *AS5047D_Handler_);

#define AS5047D_Check_MAG_TooLow(DIAAGC)      ((DIAAGC >> 11) & 0x0001)
#define AS5047D_Check_MAG_TooHigh(DIAAGC)     ((DIAAGC >> 10) & 0x0001)
#define AS5047D_Check_CORDIC_Overflow(DIAAGC) ((DIAAGC >> 9)  & 0x0001)
#define AS5047D_Check_LF_finished(DIAAGC)     ((DIAAGC >> 8)  & 0x0001)

#endif /* AS5047D_H_ */
