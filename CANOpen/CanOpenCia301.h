#pragma once

#include <stdint.h>

#include "CanOpenDef.h"
#include "CanOpenCia301Def.h"

#define	MAX_NUMBER_OF_ASSIGNED_PDO_MAPPING_OBJECT	5
#define	MAX_NUMBER_OF_PDO_MAPPING_OBJECT			10

#define	MAX_NUMBER_OF_SYNC_MANAGER					8

typedef int8_t (*SetInt8DataFunc)(uint16_t index, uint8_t subIndex, int8_t data);
typedef int8_t (*SetUint8DataFunc)(uint16_t index, uint8_t subIndex, uint8_t data);
typedef int8_t (*SetInt16DataFunc)(uint16_t index, uint8_t subIndex, int16_t data);
typedef int8_t (*SetUint16DataFunc)(uint16_t index, uint8_t subIndex, uint16_t data);
typedef int8_t (*SetInt32DataFunc)(uint16_t index, uint8_t subIndex, int32_t data);
typedef int8_t (*SetUint32DataFunc)(uint16_t index, uint8_t subIndex, uint32_t data);

class CanOpenCia301
{
public:
	CanOpenCia301(uint32_t vendorId, uint32_t productCode, uint32_t revisionNumber, uint32_t serialNumber);
	
public:
	uint32_t _deviceType;
	uint8_t _errorReg;
	
	uint32_t _vendorId;
	uint32_t _productCode;
	uint32_t _revisionNumber;
	uint32_t _serialNumber;
	
	uint32_t _id;
	
	uint32_t StoreAllParameters(uint32_t data);
	
	uint8_t _numberOfSyncManagerChannels;
	uint8_t _commTypeSyncManagerChannel[MAX_NUMBER_OF_SYNC_MANAGER];
	
	uint8_t _syncMgrCh2NumberOfMappingObject;
	uint16_t _syncMgrCh2MappingObject[MAX_NUMBER_OF_ASSIGNED_PDO_MAPPING_OBJECT];
	
	uint8_t _syncMgrCh3NumberOfMappingObject;
	uint16_t _syncMgrCh3MappingObject[MAX_NUMBER_OF_ASSIGNED_PDO_MAPPING_OBJECT];
	
	uint8_t _numberOfRxPdo1MapingObject;
	uint32_t _rxPdo1MappingObject[MAX_NUMBER_OF_PDO_MAPPING_OBJECT];
	uint8_t _numberOfRxPdo2MapingObject;
	uint32_t _rxPdo2MappingObject[MAX_NUMBER_OF_PDO_MAPPING_OBJECT];
	uint8_t _numberOfRxPdo3MapingObject;
	uint32_t _rxPdo3MappingObject[MAX_NUMBER_OF_PDO_MAPPING_OBJECT];
	uint8_t _numberOfRxPdo4MapingObject;
	uint32_t _rxPdo4MappingObject[MAX_NUMBER_OF_PDO_MAPPING_OBJECT];
	uint8_t _numberOfRxPdo5MapingObject;
	uint32_t _rxPdo5MappingObject[MAX_NUMBER_OF_PDO_MAPPING_OBJECT];
	
	uint8_t _numberOfTxPdo1MapingObject;
	uint32_t _txPdo1MappingObject[MAX_NUMBER_OF_PDO_MAPPING_OBJECT];
	uint8_t _numberOfTxPdo2MapingObject;
	uint32_t _txPdo2MappingObject[MAX_NUMBER_OF_PDO_MAPPING_OBJECT];
	uint8_t _numberOfTxPdo3MapingObject;
	uint32_t _txPdo3MappingObject[MAX_NUMBER_OF_PDO_MAPPING_OBJECT];
	uint8_t _numberOfTxPdo4MapingObject;
	uint32_t _txPdo4MappingObject[MAX_NUMBER_OF_PDO_MAPPING_OBJECT];
	uint8_t _numberOfTxPdo5MapingObject;
	uint32_t _txPdo5MappingObject[MAX_NUMBER_OF_PDO_MAPPING_OBJECT];
	
	uint32_t SetSyncMgrCh2NumberOfMappingObject(uint8_t syncMgrCh2NumberOfMappingObject);
	uint32_t SetSyncMgrCh2MappingObject(uint8_t index, uint16_t mappingObject);
	uint16_t GetSyncMgrCh2MappingObject(uint8_t index);
	
	uint32_t SetSyncMgrCh3NumberOfMappingObject(uint8_t syncMgrCh3NumberOfMappingObject);
	uint32_t SetSyncMgrCh3MappingObject(uint8_t index, uint16_t mappingObject);
	uint16_t GetSyncMgrCh3MappingObject(uint8_t index);
	
	//	Callback Function
	SetInt8DataFunc _setInt8DataFunc;
	SetInt16DataFunc _setInt16DataFunc;
	SetInt32DataFunc _setInt32DataFunc;
	SetUint8DataFunc _setUint8DataFunc;
	SetUint16DataFunc _setUint16DataFunc;
	SetUint32DataFunc _setUint32DataFunc;
};
