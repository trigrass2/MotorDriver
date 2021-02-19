#include <string.h>
#include "main.h"
#include "utils.h"
#include "stm32_hal_legacy.h"
//#include "stm32H7xx_hal_flash_ex.h"

// Base address of the Flash sectors
#if 0
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) // Base address of Sector 0, 16 Kbytes
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) // Base address of Sector 1, 16 Kbytes
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) // Base address of Sector 2, 16 Kbytes
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) // Base address of Sector 3, 16 Kbytes
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) // Base address of Sector 4, 64 Kbytes
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) // Base address of Sector 5, 128 Kbytes
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) // Base address of Sector 6, 128 Kbytes
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) // Base address of Sector 7, 128 Kbytes
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) // Base address of Sector 8, 128 Kbytes
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) // Base address of Sector 9, 128 Kbytes
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) // Base address of Sector 10, 128 Kbytes
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) // Base address of Sector 11, 128 Kbytes
#else
/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */
#define FLASH_END_ADDR				   ((uint32_t)0x081F0000) /* Base @ of Sector 7, 128 Kbytes */

#endif


#define	PROPERTIES_MEMORY_ADDR		ADDR_FLASH_SECTOR_0_BANK2

uint8_t *propertiyesMemoryAddr = (uint8_t *)PROPERTIES_MEMORY_ADDR;

/**
  * @brief  Gets the sector of a given address
  * @param  Address Address of the FLASH Memory
  * @retval The sector of a given address
  */
uint32_t GetSector(uint32_t Address) //pjg++190425
	{
	uint32_t sector = 0;

	if(((Address < ADDR_FLASH_SECTOR_1_BANK1) && (Address >= ADDR_FLASH_SECTOR_0_BANK1)) || \
		((Address < ADDR_FLASH_SECTOR_1_BANK2) && (Address >= ADDR_FLASH_SECTOR_0_BANK2)))    
	{
		sector = FLASH_SECTOR_0;  
	}
	else if(((Address < ADDR_FLASH_SECTOR_2_BANK1) && (Address >= ADDR_FLASH_SECTOR_1_BANK1)) || \
		((Address < ADDR_FLASH_SECTOR_2_BANK2) && (Address >= ADDR_FLASH_SECTOR_1_BANK2)))    
	{
		sector = FLASH_SECTOR_1;  
	}
	else if(((Address < ADDR_FLASH_SECTOR_3_BANK1) && (Address >= ADDR_FLASH_SECTOR_2_BANK1)) || \
		((Address < ADDR_FLASH_SECTOR_3_BANK2) && (Address >= ADDR_FLASH_SECTOR_2_BANK2)))    
	{
		sector = FLASH_SECTOR_2;  
	}
	else if(((Address < ADDR_FLASH_SECTOR_4_BANK1) && (Address >= ADDR_FLASH_SECTOR_3_BANK1)) || \
		((Address < ADDR_FLASH_SECTOR_4_BANK2) && (Address >= ADDR_FLASH_SECTOR_3_BANK2)))    
	{
		sector = FLASH_SECTOR_3;  
	}
	else if(((Address < ADDR_FLASH_SECTOR_5_BANK1) && (Address >= ADDR_FLASH_SECTOR_4_BANK1)) || \
		((Address < ADDR_FLASH_SECTOR_5_BANK2) && (Address >= ADDR_FLASH_SECTOR_4_BANK2)))    
	{
		sector = FLASH_SECTOR_4;  
	}
	else if(((Address < ADDR_FLASH_SECTOR_6_BANK1) && (Address >= ADDR_FLASH_SECTOR_5_BANK1)) || \
		((Address < ADDR_FLASH_SECTOR_6_BANK2) && (Address >= ADDR_FLASH_SECTOR_5_BANK2)))    
	{
		sector = FLASH_SECTOR_5;  
	}
	else if(((Address < ADDR_FLASH_SECTOR_7_BANK1) && (Address >= ADDR_FLASH_SECTOR_6_BANK1)) || \
		((Address < ADDR_FLASH_SECTOR_7_BANK2) && (Address >= ADDR_FLASH_SECTOR_6_BANK2)))    
	{
		sector = FLASH_SECTOR_6;  
	}
	else if(((Address < ADDR_FLASH_SECTOR_0_BANK2) && (Address >= ADDR_FLASH_SECTOR_7_BANK1)) || \
		((Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7_BANK2)))
	{
		sector = FLASH_SECTOR_7;  
	}
	else
	{
		sector = FLASH_SECTOR_7;
	}

	return sector;
}

int32_t LoadProperties(uint8_t *buffer, uint32_t size)
{
	uint32_t index;
	
	if(size > 131072)	size= 131072;	//	128KBytes
	
	for(index = 0; index < size; index++) {
	//for(index = 0; index < size; index+=8) {
		*(buffer + index) = *((volatile uint8_t *)(propertiyesMemoryAddr + index));
		//*(buffer + index) = *((uint64_t*)propertiyesMemoryAddr+index/8);
	}
	
	return size;
}

int32_t SaveProperties(uint8_t *buffer, uint32_t size)
{
	FLASH_EraseInitTypeDef erase_sector;
	uint32_t error;
	uint32_t index;
	//uint32_t data;
	uint32_t Address;


	//	Erase Flash Memory
	HAL_FLASH_Unlock();
	//HAL_FLASH_OB_Unlock();
	
	erase_sector.TypeErase     = FLASH_TYPEERASE_SECTORS;
	erase_sector.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	erase_sector.Banks         = FLASH_BANK_2;
	erase_sector.Sector        = GetSector(PROPERTIES_MEMORY_ADDR);//FLASH_SECTOR_0;
	erase_sector.NbSectors     = 1;

	if(HAL_FLASHEx_Erase(&erase_sector, &error) != HAL_OK) {
		HAL_FLASH_OB_Lock();
		HAL_FLASH_Lock ();
		
		return -1;
	}
	
	//HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock ();
		
	if(size > 131072)	size= 131072;	//	128KBytes

	HAL_FLASH_Unlock();
	//HAL_FLASH_OB_Unlock();
	Address = PROPERTIES_MEMORY_ADDR;
	for(index = 0; index < size; index+=32) {
		if(HAL_FLASH_Program (FLASH_TYPEPROGRAM_FLASHWORD, Address + index,  (uint64_t)((uint32_t)buffer+index)) != HAL_OK) {
			//HAL_FLASH_OB_Lock();
			HAL_FLASH_Lock ();
			return -1;
		}
	}

	//HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock ();

	return size;
}
