#include <string.h>
#include "stm32f4xx_hal.h"
#include "utils.h"

// Base address of the Flash sectors
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



#define	PROPERTIES_MEMORY_ADDR		ADDR_FLASH_SECTOR_11

uint8_t *propertiyesMemoryAddr = (uint8_t *)PROPERTIES_MEMORY_ADDR;

int32_t LoadProperties(uint8_t *buffer, uint32_t size)
{
	uint32_t index;
	
	if(size > 131072)	size= 131072;	//	128KBytes
	
	for(index = 0; index < size; index++) {
		*(buffer + index) = *((volatile uint8_t *)(propertiyesMemoryAddr + index));
	}
	
	return size;
}

int32_t SaveProperties(uint8_t *buffer, uint32_t size)
{
	FLASH_EraseInitTypeDef erase_sector;
	uint32_t error;
	uint32_t index;
	//uint32_t data;


	//	Erase Flash Memory
	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();
	
	erase_sector.TypeErase = TYPEERASE_SECTORS;
	erase_sector.VoltageRange = VOLTAGE_RANGE_3;
	erase_sector.Sector = FLASH_SECTOR_11;
	erase_sector.NbSectors = 1;

	if(HAL_FLASHEx_Erase(&erase_sector, &error) != HAL_OK) {
		HAL_FLASH_OB_Lock();
		HAL_FLASH_Lock ();
		
		return -1;
	}
	
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock ();
		
	if(size > 131072)	size= 131072;	//	128KBytes

	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();
	
	for(index = 0; index < size; index++) {
		if(HAL_FLASH_Program (TYPEPROGRAM_BYTE, PROPERTIES_MEMORY_ADDR + index, *(buffer + index)) != HAL_OK) {
			HAL_FLASH_OB_Lock();
			HAL_FLASH_Lock ();
			return -1;
		}
	}

	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock ();

	return size;
}
