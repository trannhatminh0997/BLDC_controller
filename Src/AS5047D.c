/*
 * AS5047D.c
 *
 *  Created on: 24. avg. 2016
 *      Author: user
 */

#include "AS5047D.h"
#include <stdint.h>
//#include "spi.h"
//#include "stm32f4xx_hal_gpio.c"


uint16_t _txData;
uint16_t _rxData = 0;
uint16_t _errCount = 0;


#define cDelay 10

void delayCycle(uint16_t cycle)
{
	for(uint16_t i = 0;i<cycle;i++);

}


uint16_t parity(uint16_t x)
{
	uint16_t parity = 0;

	while(x != 0)
	{
		parity ^= x;
		x >>= 1;
	}

	return (parity & 0x1);
}



HAL_StatusTypeDef AS5047D_Read(AS5047D_Handler *AS5047D_Handler_, uint16_t address,uint16_t* pdata)
{
	if (parity(address | AS5047D_RD) == 1) address = address | 0x8000; // set parity bit
	address = address | AS5047D_RD; // it's a read command
	_txData = address;
	
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_RESET);
	delayCycle(cDelay);
	if (HAL_SPI_TransmitReceive(AS5047D_Handler_->hspi_Handler, (uint8_t*)&_txData,(uint8_t*)&_rxData,1,2000) != HAL_OK)
	{
		//Error_Handler();
	}
	while(HAL_SPI_GetState(AS5047D_Handler_->hspi_Handler)!= HAL_SPI_STATE_READY);
	delayCycle(cDelay);
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_SET);
	delayCycle(cDelay);
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_RESET);
	delayCycle(cDelay);

	if (HAL_SPI_TransmitReceive(AS5047D_Handler_->hspi_Handler, (uint8_t*)&_txData,(uint8_t*)&_rxData,1,2000) != HAL_OK)
	{
		//Error_Handler();
	}
	
	while(HAL_SPI_GetState(AS5047D_Handler_->hspi_Handler)!= HAL_SPI_STATE_READY);
	delayCycle(cDelay);
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_SET);

	if((parity(_rxData) == 1)||((_rxData&EF) != 0)) //check parity bit 
	{
		//while (1);
		//_errCount++;
		return HAL_ERROR;
	}
	else 
	{
		_rxData = _rxData & 0x3FFF;  // filter bits outside data, strip bit 14..15
		*pdata =  _rxData;
		return HAL_OK;
	}
}

HAL_StatusTypeDef AS5047D_Write(AS5047D_Handler *AS5047D_Handler_, uint16_t address, uint16_t data)
{
	if (parity(address & 0x3FFF) == 1) address = address | 0x8000; // set parity bit
	//address = address & (WR | 0x8000);  // its  a write command and don't change the parity bit (0x8000)
	
	
	_txData = address;
	
	
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_RESET);
	delayCycle(cDelay);
	
	if (HAL_SPI_TransmitReceive(AS5047D_Handler_->hspi_Handler,(uint8_t*)&_txData,(uint8_t*)&_rxData,1,1000) != HAL_OK)
	{
		return HAL_ERROR;
	}
	delayCycle(cDelay);
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_SET);
	delayCycle(cDelay);
	
	
	if (parity(data & 0x3FFF) == 1) data = data | 0x8000; // set parity bit
	//data = data & (WR | 0x8000); // its a write command and don't change the parity bit (0x8000)
	_txData = data;
	
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_RESET);
	delayCycle(cDelay);

	
	if (HAL_SPI_TransmitReceive(AS5047D_Handler_->hspi_Handler,(uint8_t*)&_txData,(uint8_t*)&_rxData,1,1000) != HAL_OK)
	{
		return HAL_ERROR;
	}
	delayCycle(cDelay);
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_SET);
	return HAL_OK;
}

HAL_StatusTypeDef AS5047D_Write_ReadPre(AS5047D_Handler *AS5047D_Handler_, uint16_t address, uint16_t data,uint16_t* preData)
{
	if (parity(address & 0x3FFF) == 1) address = address | 0x8000; // set parity bit
	//address = address & (WR | 0x8000);  // its  a write command and don't change the parity bit (0x8000)
	
	
	_txData = address;
	
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_RESET);
	delayCycle(cDelay);
	
	if (HAL_SPI_TransmitReceive(AS5047D_Handler_->hspi_Handler,(uint8_t*)&_txData,(uint8_t*)&_rxData,1,1000) != HAL_OK)
	{
		return HAL_ERROR;
	}
	delayCycle(cDelay);
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_SET);
	delayCycle(cDelay);
	
	if (parity(data & 0x3FFF) == 1) data = data | 0x8000; // set parity bit
	//data = data & (WR | 0x8000); // its a write command and don't change the parity bit (0x8000)
	_txData = data;
	
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_RESET);
	delayCycle(cDelay);

	
	if (HAL_SPI_TransmitReceive(AS5047D_Handler_->hspi_Handler,(uint8_t*)&_txData,(uint8_t*)&_rxData,1,1000) != HAL_OK)
	{
		return HAL_ERROR;
	}
	*preData = _rxData & 0x3FFF;
	delayCycle(cDelay);
	HAL_GPIO_WritePin(AS5047D_Handler_->nssPort, AS5047D_Handler_->nssPin, GPIO_PIN_SET);
	return HAL_OK;
}



uint16_t err_code = 0;
void AS5047D_Check_Transmission_Error(AS5047D_Handler *AS5047D_Handler_)
{
	/** Check if transmission error **/
	AS5047D_Read(AS5047D_Handler_, AS5047D_ERRFL,&err_code);
//	if(AS5047D_Read(AS5047D_Handler_, AS5047D_ERRFL) != 0)
	if(err_code != 0)
	{
		while (1);
	}
}

void AS5047D_SetZero(AS5047D_Handler *AS5047D_Handler_)
{
	uint16_t DIAAGC=0;
	/** Check diagnostics reg **/
	AS5047D_Read(AS5047D_Handler_, AS5047D_DIAAGC,&DIAAGC);
	//AS5047D_Check_Transmission_Error();
	//if((AS5047D_Check_MAG_TooLow(DIAAGC)) || (AS5047D_Check_MAG_TooHigh(DIAAGC)) || (AS5047D_Check_CORDIC_Overflow(DIAAGC)) || !(AS5047D_Check_LF_finished(DIAAGC)))
	//{
		//Error_Handler();
	//}

	/** Get uncompensated angle reg value **/
	uint16_t ANGLEUNC = 0;
	 AS5047D_Read(AS5047D_Handler_, AS5047D_ANGLEUNC,&ANGLEUNC);
	//AS5047D_Check_Transmission_Error();

	/** Write to zero pos regs **/
	AS5047D_Write(AS5047D_Handler_ , AS5047D_ZPOSM, (ANGLEUNC >> 6) & 0x00FF);
	//AS5047D_Check_Transmission_Error();
	AS5047D_Write(AS5047D_Handler_ , AS5047D_ZPOSL, ANGLEUNC & 0x003F);
	//AS5047D_Check_Transmission_Error();
}

uint16_t AS5047D_GetZero(AS5047D_Handler *AS5047D_Handler_)
{
	uint16_t ZPOSM = 0;
	uint16_t ZPOSL = 0;

	AS5047D_Read(AS5047D_Handler_, AS5047D_ZPOSM,&ZPOSM);
	//AS5047D_Check_Transmission_Error();
	AS5047D_Read(AS5047D_Handler_, AS5047D_ZPOSL,&ZPOSL);
	//AS5047D_Check_Transmission_Error();

	return (((ZPOSM << 6) & 0x3FC0) | (ZPOSL & 0x003F));
}

uint8_t AS5047D_Get_AGC_Value(AS5047D_Handler *AS5047D_Handler_)
{
	/** Read diagnostics reg **/
	uint16_t DIAAGC;
	AS5047D_Read(AS5047D_Handler_, AS5047D_DIAAGC,&DIAAGC);
	//AS5047D_Check_Transmission_Error();
	return (uint8_t)((DIAAGC >> 8) & 0x00FF);
}

void AS5047D_Init(AS5047D_Handler *AS5047D_Handler_)
{

	/* Initiaize AS5047D */
	AS5047D_Write(AS5047D_Handler_ , AS5047D_SETTINGS1,0x05); //0b00000101);
	AS5047D_Check_Transmission_Error(AS5047D_Handler_);
	AS5047D_Write(AS5047D_Handler_ , AS5047D_SETTINGS2,0x00); //0b00000000);
	AS5047D_Check_Transmission_Error(AS5047D_Handler_);
}

uint16_t AS5047D_Get_CORDICMAG_Value(AS5047D_Handler *AS5047D_Handler_)
{
	uint16_t CORDICMAG;
	AS5047D_Read(AS5047D_Handler_, AS5047D_CORDICMAG,&CORDICMAG);
	//AS5047D_Check_Transmission_Error();
	return CORDICMAG;
}

uint16_t AS5047D_Get_ANGLEUNC_Value(AS5047D_Handler *AS5047D_Handler_)
{
	uint16_t ANGLEUNC;
	AS5047D_Read(AS5047D_Handler_, AS5047D_ANGLEUNC,&ANGLEUNC);
	//AS5047D_Check_Transmission_Error();
	return ANGLEUNC;
}

uint16_t AS5047D_Get_ANGLECOM_Value(AS5047D_Handler *AS5047D_Handler_)
{
	uint16_t ANGLECOM;
	AS5047D_Read(AS5047D_Handler_, AS5047D_ANGLECOM,&ANGLECOM);
	//AS5047D_Check_Transmission_Error();
	return ANGLECOM;
}

float AS5047D_Get_True_Angle_Value(AS5047D_Handler *AS5047D_Handler_)
{
	return((float)AS5047D_Get_ANGLEUNC_Value(AS5047D_Handler_) * 360.0f / 16383.0f);
	//return((float)AS5047D_Get_ANGLECOM_Value() * 360.0f / 16383.0f);
}
