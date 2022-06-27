#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "stdint.h"
#include "string.h"

void deleteBuffer(char* data);
void 	Flash_Lock(void);
void 	Flash_Unlock(void);
void 	Flash_Erase(uint32_t addr);
void Flash_Write_HalfWord(uint32_t addr, uint16_t data);
uint16_t Flash_Read_Int(uint32_t addr);
void 	Flash_Write_Char(uint32_t addr, char* data);
void 	Flash_ReadChar(char* dataOut, uint32_t addr1, uint32_t addr2);
void 	Flash_ProgramPage(char* dataIn, uint32_t addr1, uint32_t addr2);


void Flash_Read_Obj(uint32_t StartAddrToReadFlash, uint32_t dataAddr, uint32_t length);
void Flash_Write_Obj(uint32_t WriteStartAddr, uint32_t dataAddr, uint32_t length);



#define FlashPageSize           1024

#define LastPageAdrr 0x0800FC00

#define DATA_START_ADDRESS 		 	((uint32_t)0x0801FC00)	//Page 127
#define LENGTH_START_ADDRESS 		((uint32_t)0x0801F810)	//Page 126

#define AddrPage1 				  		((uint32_t)0x08000000)
#define AddrPage127 						((uint32_t)0x0801FC00)
#define AddrPage126 						((uint32_t)0x0801F810)	//Page 126
