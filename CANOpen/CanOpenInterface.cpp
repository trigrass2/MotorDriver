#include "CanOpenInterface.h"
#include "CanOpen.h"
#include "utils.h"

extern CanOpen canOpen;

int8_t ProcessRxPDO(uint8_t *data, int8_t size)
{
	uint16_t syncManagerCh2PdoMappingObject = canOpen.GetSyncMgrCh2MappingObject(0);
	
	if(syncManagerCh2PdoMappingObject == CIA_301_RX_PDO_MAPPING1) {
		if(size != 10)	return -1;
		
		return ProcessRxPDO1(	BytesToUint16(data),		//	2Bytes : ControlWord
							 	BytesToInt32((data+2)),		//	4Bytes : TargetPosition
								BytesToUint32((data+6)));	//	4Bytes : DigitalOutput
	}
	else if(syncManagerCh2PdoMappingObject == CIA_301_RX_PDO_MAPPING2) {
		if(size != 12)	return -1;
		return ProcessRxPDO2(	BytesToUint16(data),		//	2Bytes : ControlWord
							 	BytesToInt32((data+2)),		//	4Bytes : TargetPosition
								BytesToInt16((data+6)),		//	2Bytes : TorqueOffset
								BytesToUint32((data+8)));	//	4Bytes : DigitalOutput
	}
	else if(syncManagerCh2PdoMappingObject == CIA_301_RX_PDO_MAPPING3) {
		if(size != 10)	return -1;
		return ProcessRxPDO3(	BytesToUint16(data),		//	2Bytes : ControlWord
							 	BytesToInt32((data+2)),		//	4Bytes : TargetVelocity
							 	BytesToUint32((data+6)));	//	4Bytes : DigitalOutput
	}
	else if(syncManagerCh2PdoMappingObject == CIA_301_RX_PDO_MAPPING4) {
		if(size != 8)	return -1;
		return ProcessRxPDO4(	BytesToUint16(data),		//	2Bytes : ControlWord
							 	BytesToInt16((data+2)),		//	2Bytes : TargetTorque
								BytesToUint32((data+4)));	//	4Bytes : DigitalOutput
	}
	else if(syncManagerCh2PdoMappingObject == CIA_301_RX_PDO_MAPPING5) {
		if(size != 17)	return -1;
		return ProcessRxPDO5(	BytesToUint16(data),		//	2Bytes : ControlWord
							 	BytesToInt32((data+2)),		//	1Byte  : ModesOfDisplay
								BytesToInt32((data+3)),		//	4Bytes : TargetPosition
								BytesToInt32((data+7)),		//	4Bytes : TargetVelocity
								BytesToInt32((data+11)),	//	2Bytes : TargetTorque
								BytesToUint32((data+13)));	//	4Bytes : DigitalOutput
	}
	else {
		return -1;
	}
}

int8_t ProcessRxPDO1(uint16_t controlWord, int32_t targetPosition, uint32_t digitalOutput)
{
	int8_t ret = 0;
	
	if(canOpen.SetControlWord(controlWord) != CAN_OPEN_ABORT_CODE_NO_ERROR)			ret = -1;
	if(canOpen.SetTargetPosition(targetPosition) != CAN_OPEN_ABORT_CODE_NO_ERROR)	ret = -1;
	if(canOpen.SetDigitalOutput(digitalOutput) != CAN_OPEN_ABORT_CODE_NO_ERROR)		ret = -1;

	return ret;
}

int8_t ProcessRxPDO2(uint16_t controlWord, int32_t targetPosition, int16_t torqueOffset, uint32_t digitalOutput)
{
	int8_t ret = 0;
	
	if(canOpen.SetControlWord(controlWord) != CAN_OPEN_ABORT_CODE_NO_ERROR)			ret = -1;
	if(canOpen.SetTargetPosition(targetPosition) != CAN_OPEN_ABORT_CODE_NO_ERROR)	ret = -1;
	if(canOpen.SetDigitalOutput(digitalOutput) != CAN_OPEN_ABORT_CODE_NO_ERROR)		ret = -1;

	return ret;
}

int8_t ProcessRxPDO3(uint16_t controlWord, int32_t targetVelocity, uint32_t digitalOutput)
{
	int8_t ret = 0;
	
	if(canOpen.SetControlWord(controlWord) != CAN_OPEN_ABORT_CODE_NO_ERROR)			ret = -1;
	if(canOpen.SetTargetVelocity(targetVelocity) != CAN_OPEN_ABORT_CODE_NO_ERROR)	ret = -1;
	if(canOpen.SetDigitalOutput(digitalOutput) != CAN_OPEN_ABORT_CODE_NO_ERROR)		ret = -1;

	return ret;
}

int8_t ProcessRxPDO4(uint16_t controlWord, int16_t targetTorque, uint32_t digitalOutput)
{
	int8_t ret = 0;
	
	if(canOpen.SetControlWord(controlWord) != CAN_OPEN_ABORT_CODE_NO_ERROR)			ret = -1;
	if(canOpen.SetTargetTorque(targetTorque) != CAN_OPEN_ABORT_CODE_NO_ERROR)	ret = -1;
	if(canOpen.SetDigitalOutput(digitalOutput) != CAN_OPEN_ABORT_CODE_NO_ERROR)		ret = -1;

	return ret;
}

int8_t ProcessRxPDO5(uint16_t controlWord, int8_t modesOfOperation, int32_t targetPosition, int32_t targetVelocity, int16_t targetTorque, uint32_t digitalOutput)
{
	int8_t ret = 0;
	
	if(canOpen.SetModesOfOperation(modesOfOperation) != CAN_OPEN_ABORT_CODE_NO_ERROR) {
		ret = -1;
	}
	
	if(modesOfOperation == CIA_402_CYCLIC_SYNC_TORQUE_MODE) {
		if(canOpen.SetTargetTorque(targetTorque) != CAN_OPEN_ABORT_CODE_NO_ERROR) {
			ret = -1;
		}
	}
	else if((modesOfOperation == CIA_402_CYCLIC_SYNC_VELOCITY_MODE) || (modesOfOperation == CIA_402_PROFILE_VELOCITY_MODE) || (modesOfOperation == CIA_402_VELOCITY_MODE)) {
		if(canOpen.SetTargetVelocity(targetVelocity) != CAN_OPEN_ABORT_CODE_NO_ERROR) {
			ret = -1;
		}
	}
	else if((modesOfOperation == CIA_402_CYCLIC_SYNC_POSITION_MODE) || (modesOfOperation == CIA_402_PROFILE_POSITION_MODE) || (modesOfOperation == CIA_402_POSITION_MODE)) {
		if(canOpen.SetTargetPosition(targetPosition) != CAN_OPEN_ABORT_CODE_NO_ERROR) {
			ret = -1;
		}
	}
	
	if(canOpen.SetControlWord(controlWord) != CAN_OPEN_ABORT_CODE_NO_ERROR) {
		ret = -1;
	}
	
	if(canOpen.SetDigitalOutput(digitalOutput) != CAN_OPEN_ABORT_CODE_NO_ERROR) {
		ret = -1;
	}
	
	return ret;
}

int8_t ProcessTxPDO(uint8_t *data, int8_t size)
{
	uint16_t syncManagerCh3PdoMappingObject = canOpen.GetSyncMgrCh3MappingObject(0);
	
	if(syncManagerCh3PdoMappingObject == CIA_301_TX_PDO_MAPPING1) {
		if(size != 10)	return -1;
		
		return ProcessRxPDO1(	BytesToUint16(data),		//	2Bytes : ControlWord
							 	BytesToInt32((data+2)),		//	4Bytes : TargetPosition
								BytesToUint32((data+6)));	//	4Bytes : DigitalOutput
	}
	else if(syncManagerCh3PdoMappingObject == CIA_301_TX_PDO_MAPPING1) {
		if(size != 12)	return -1;
		return ProcessRxPDO2(	BytesToUint16(data),		//	2Bytes : ControlWord
							 	BytesToInt32((data+2)),		//	4Bytes : TargetPosition
								BytesToInt16((data+6)),		//	2Bytes : TorqueOffset
								BytesToUint32((data+8)));	//	4Bytes : DigitalOutput
	}
	else if(syncManagerCh3PdoMappingObject == CIA_301_TX_PDO_MAPPING1) {
		if(size != 10)	return -1;
		return ProcessRxPDO3(	BytesToUint16(data),		//	2Bytes : ControlWord
							 	BytesToInt32((data+2)),		//	4Bytes : TargetVelocity
							 	BytesToUint32((data+6)));	//	4Bytes : DigitalOutput
	}
	else if(syncManagerCh3PdoMappingObject == CIA_301_TX_PDO_MAPPING1) {
		if(size != 8)	return -1;
		return ProcessRxPDO4(	BytesToUint16(data),		//	2Bytes : ControlWord
							 	BytesToInt16((data+2)),		//	2Bytes : TargetTorque
								BytesToUint32((data+4)));	//	4Bytes : DigitalOutput
	}
	else if(syncManagerCh3PdoMappingObject == CIA_301_TX_PDO_MAPPING1) {
		if(size != 17)	return -1;
		return ProcessRxPDO5(	BytesToUint16(data),		//	2Bytes : ControlWord
							 	BytesToInt32((data+2)),		//	1Byte  : ModesOfDisplay
								BytesToInt32((data+3)),		//	4Bytes : TargetPosition
								BytesToInt32((data+7)),		//	4Bytes : TargetVelocity
								BytesToInt32((data+11)),	//	2Bytes : TargetTorque
								BytesToUint32((data+13)));	//	4Bytes : DigitalOutput
	}
	else {
		return -1;
	}
}

int8_t ProcessTxPDO1(uint16_t *statusWord, int32_t *actualPosition, uint32_t *digitalInput)
{
	*statusWord = canOpen._statusWord;
	*actualPosition = canOpen._actualPosition;
	*digitalInput = canOpen._digitalInput;
	
	return 0;
}

int8_t ProcessTxPDO2(uint16_t *statusWord, int32_t *actualPosition, int32_t *actualVelocity, uint32_t *digitalInput)
{
	*statusWord = canOpen._statusWord;
	*actualPosition = canOpen._actualPosition;
	*actualVelocity = canOpen._actualVelocity;
	*digitalInput = canOpen._digitalInput;
	
	return 0;
}

int8_t ProcessTxPDO3(uint16_t *statusWord, int32_t *actualPosition, int32_t *actualVelocity, int16_t *actualTorque, uint32_t *digitalInput)
{
	*statusWord = canOpen._statusWord;
	*actualPosition = canOpen._actualPosition;
	*actualVelocity = canOpen._actualVelocity;
	*actualTorque = canOpen._actualTorque;
	*digitalInput = canOpen._digitalInput;
	
	return 0;
}

int8_t ProcessTxPDO4(uint16_t *statusWord, int32_t *actualPosition, int32_t *actualVelocity, int16_t *actualCurrent, uint32_t *digitalInput)
{
	*statusWord = canOpen._statusWord;
	*actualPosition = canOpen._actualPosition;
	*actualVelocity = canOpen._actualVelocity;
	*actualCurrent = canOpen._actualCurrent;
	*digitalInput = canOpen._digitalInput;
	
	return 0;
}

int8_t ProcessTxPDO5(uint16_t *statusWord, int8_t *modesOfOperationDisplay, int32_t *actualPosition, int32_t *actualVelocity, int16_t *actualTorque, uint32_t *digitalInput)
{
	*statusWord = canOpen._statusWord;
	*modesOfOperationDisplay = canOpen._modesOfOperationDisplay;
	*actualPosition = canOpen._actualPosition;
	*actualVelocity = canOpen._actualVelocity;
	*actualTorque = canOpen._actualTorque;
	*digitalInput = canOpen._digitalInput;
	
	return 0;
}