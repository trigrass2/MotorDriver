#pragma once

#include "CanOpenCia402.h"

class CanOpen : public CanOpenCia402
{
public:
	CanOpen(uint32_t vendorId, uint32_t productCode, uint32_t revisionNumber, uint32_t serialNumber);
	
public:
	int8_t ProcessSdo(CAN_OPEN_SDO_MSG *canOpenSdoMsg);
	//int8_t ProcessSdoCanOpen(uint16_t &id, CAN_OPEN_SDO_MSG &canOpenSdoMsg);
	int8_t ProcessSdoCanOpen(uint32_t *id, uint8_t *data, uint32_t *len);
		
};