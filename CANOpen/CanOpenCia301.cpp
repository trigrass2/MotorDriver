#include "CanOpenCia301.h"

CanOpenCia301::CanOpenCia301(uint32_t vendorId, uint32_t productCode, uint32_t revisionNumber, uint32_t serialNumber)
: _vendorId(vendorId), _productCode(productCode), _revisionNumber(revisionNumber), _serialNumber(serialNumber)
{
	_errorReg = 0x00;
	
	_vendorId = 0x00000892;
	_productCode = 0x000000FF;
	_revisionNumber = 0x20161010;
	_serialNumber = 0x20161010;
	
	_id = 2;
	
	_numberOfSyncManagerChannels = MAX_NUMBER_OF_SYNC_MANAGER;
	for(int8_t i = 4; i < MAX_NUMBER_OF_SYNC_MANAGER; i++) {
		_commTypeSyncManagerChannel[i] = CIA_301_COMM_TYPE_NOT_USED;
	}
	_commTypeSyncManagerChannel[0] = CIA_301_COMM_TYPE_MAILBOX_RECEPTION;
	_commTypeSyncManagerChannel[1] = CIA_301_COMM_TYPE_MAILBOX_TRANSMISSION;
	_commTypeSyncManagerChannel[2] = CIA_301_COMM_TYPE_MAILBOX_PROCESS_DATA_OUTPUT;
	_commTypeSyncManagerChannel[3] = CIA_301_COMM_TYPE_MAILBOX_PROCESS_DATA_INPUT;
	
	for(int8_t i = 1; i < MAX_NUMBER_OF_ASSIGNED_PDO_MAPPING_OBJECT; i++) {
		_syncMgrCh2MappingObject[i] = 0x0000;
		_syncMgrCh3MappingObject[i] = 0x0000;
	}
	
	_syncMgrCh2NumberOfMappingObject = 1;
	_syncMgrCh2MappingObject[0] = CIA_301_RX_PDO_MAPPING5;
	
	_syncMgrCh3NumberOfMappingObject = 1;
	_syncMgrCh3MappingObject[0] = CIA_301_TX_PDO_MAPPING5;

	for(int8_t i = 0; i < MAX_NUMBER_OF_PDO_MAPPING_OBJECT; i++) {
		_rxPdo1MappingObject[i] = 0x00000000;
		_rxPdo2MappingObject[i] = 0x00000000;
		_rxPdo3MappingObject[i] = 0x00000000;
		_rxPdo4MappingObject[i] = 0x00000000;
		_rxPdo5MappingObject[i] = 0x00000000;
		
		_txPdo1MappingObject[i] = 0x00000000;
		_txPdo2MappingObject[i] = 0x00000000;
		_txPdo3MappingObject[i] = 0x00000000;
		_txPdo4MappingObject[i] = 0x00000000;
		_txPdo5MappingObject[i] = 0x00000000;
	}
	
	//	RxPDO Mapping
	_numberOfRxPdo1MapingObject = 3;
	_rxPdo1MappingObject[0] = 0x60400010;
	_rxPdo1MappingObject[1] = 0x607A0020;
	_rxPdo1MappingObject[2] = 0x60FE0020;
	
	_numberOfRxPdo2MapingObject = 4;
	_rxPdo2MappingObject[0] = 0x60400010;
	_rxPdo2MappingObject[1] = 0x607A0020;
	_rxPdo2MappingObject[2] = 0x60B20010;
	_rxPdo2MappingObject[3] = 0x60FE0020;
	
	_numberOfRxPdo3MapingObject = 3;
	_rxPdo3MappingObject[0] = 0x60400010;
	_rxPdo3MappingObject[1] = 0x60FF0020;
	_rxPdo3MappingObject[2] = 0x60FE0020;
	
	_numberOfRxPdo4MapingObject = 3;
	_rxPdo4MappingObject[0] = 0x60400010;
	_rxPdo4MappingObject[1] = 0x60710010;
	_rxPdo4MappingObject[2] = 0x60FE0020;
	
	_numberOfRxPdo5MapingObject = 6;
	_rxPdo5MappingObject[0] = 0x60400010;
	_rxPdo5MappingObject[1] = 0x60600008;
	_rxPdo5MappingObject[2] = 0x607A0020;
	_rxPdo5MappingObject[3] = 0x60FF0020;
	_rxPdo5MappingObject[4] = 0x60710010;
	_rxPdo5MappingObject[5] = 0x60FE0020;
	
	//	TxPDO Mapping
	_numberOfTxPdo1MapingObject = 3;
	_txPdo1MappingObject[0] = 0x60410010;
	_txPdo1MappingObject[1] = 0x60640020;
	_txPdo1MappingObject[2] = 0x60FD0020;
	
	_numberOfTxPdo2MapingObject = 4;
	_txPdo2MappingObject[0] = 0x60410010;
	_txPdo2MappingObject[1] = 0x60640020;
	_txPdo2MappingObject[2] = 0x606C0020;
	_txPdo2MappingObject[3] = 0x60FD0020;
	
	_numberOfTxPdo3MapingObject = 5;
	_txPdo3MappingObject[0] = 0x60410010;
	_txPdo3MappingObject[1] = 0x60640020;
	_txPdo3MappingObject[2] = 0x606C0020;
	_txPdo3MappingObject[3] = 0x60770010;
	_txPdo3MappingObject[4] = 0x60FD0020;
	
	_numberOfTxPdo4MapingObject = 5;
	_txPdo4MappingObject[0] = 0x60410010;
	_txPdo4MappingObject[1] = 0x60640020;
	_txPdo4MappingObject[2] = 0x606C0020;
	_txPdo4MappingObject[3] = 0x60780010;
	_txPdo4MappingObject[4] = 0x60FD0020;
	
	_numberOfTxPdo5MapingObject = 6;
	_txPdo5MappingObject[0] = 0x60410010;
	_txPdo5MappingObject[1] = 0x60610008;
	_txPdo5MappingObject[2] = 0x60640020;
	_txPdo5MappingObject[3] = 0x606C0020;
	_txPdo5MappingObject[4] = 0x60770010;
	_txPdo5MappingObject[5] = 0x60FD0020;
}

uint32_t CanOpenCia301::StoreAllParameters(uint32_t data)
{
	if(data != ASCII_SAVE) {
		return CAN_OPEN_ABORT_CODE_ILLEGAL_COMMAND_ERROR;
	}
	
	if(_setUint32DataFunc(CIA_301_STORE, CIA_301_STORE_ALL_PARAMETERS, data) < 0) {
		return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
	}
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia301::SetSyncMgrCh2NumberOfMappingObject(uint8_t syncMgrCh2NumberOfMappingObject)
{
	if(syncMgrCh2NumberOfMappingObject != 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_syncMgrCh2NumberOfMappingObject = syncMgrCh2NumberOfMappingObject;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia301::SetSyncMgrCh2MappingObject(uint8_t index, uint16_t mappingObject)
{
	if(index >= MAX_NUMBER_OF_ASSIGNED_PDO_MAPPING_OBJECT) {
		return CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR;
	}
	
	if((mappingObject < CIA_301_RX_PDO_MAPPING1) || (mappingObject > CIA_301_RX_PDO_MAPPING5)) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_syncMgrCh2MappingObject[index] = mappingObject;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint16_t CanOpenCia301::GetSyncMgrCh2MappingObject(uint8_t index)
{
	if(index >= MAX_NUMBER_OF_ASSIGNED_PDO_MAPPING_OBJECT) {
		return 0x0000;
	}
	
	return _syncMgrCh2MappingObject[index];
}

uint32_t CanOpenCia301::SetSyncMgrCh3NumberOfMappingObject(uint8_t syncMgrCh3NumberOfMappingObject)
{
	if(syncMgrCh3NumberOfMappingObject != 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_syncMgrCh3NumberOfMappingObject = syncMgrCh3NumberOfMappingObject;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia301::SetSyncMgrCh3MappingObject(uint8_t index, uint16_t mappingObject)
{
	if(index >= MAX_NUMBER_OF_ASSIGNED_PDO_MAPPING_OBJECT) {
		return CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR;
	}
	
	if((mappingObject < CIA_301_TX_PDO_MAPPING1) || (mappingObject > CIA_301_TX_PDO_MAPPING5)) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_syncMgrCh3MappingObject[index] = mappingObject;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint16_t CanOpenCia301::GetSyncMgrCh3MappingObject(uint8_t index)
{
	if(index >= MAX_NUMBER_OF_ASSIGNED_PDO_MAPPING_OBJECT) {
		return 0x0000;
	}
	
	return _syncMgrCh3MappingObject[index];
}
