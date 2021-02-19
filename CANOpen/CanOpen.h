#pragma once

#include "CanOpenCia402.h"

/*pjg++181112
CiA = CAN in Automation
* CiA 301 version 4.2.0
CANopen application layer and communication profile
* CiA DSP-302
Framework for Programmable Devices
* CiA DSP-304
Framework for Safety-Relevant Communication
* CiA DSP-305 V1.1 
* CiA DSP-401 v2.0 
* CiA 402-1 version 4.0.0
CANopen device profile for drives and motion control ? Part 1: General definitions 
* CiA 402-2 version 4.1.0
CANopen device profile for drives and motion control ? Part 2: Operation modes and application data 
* CiA 402-3 version 4.0.0
CANopen device profile for drives and motion control ? Part 3: PDO mapping 
* CiA 402-4 version 1.0.0
CANopen device profile for drives and motion control ? Part 4: Safety functionality 
* CiA 402-5 version 1.0.0
CANopen device profile for drives and motion control ? Part 5: PDO mapping superset 
* CiA 402-6 version 1.0.0
CANopen device profile for drives and motion control ? Part 6: CANopen FD PDO mapping 
*/

class CanOpen : public CanOpenCia402
{
public:
	CanOpen(uint32_t vendorId, uint32_t productCode, uint32_t revisionNumber, uint32_t serialNumber);
	
public:
	int8_t ProcessSdo(CAN_OPEN_SDO_MSG *canOpenSdoMsg);
	//int8_t ProcessSdoCanOpen(uint16_t &id, CAN_OPEN_SDO_MSG &canOpenSdoMsg);
	int8_t ProcessSdoCanOpen(uint32_t *id, uint8_t *data, uint32_t *len);
		
};
