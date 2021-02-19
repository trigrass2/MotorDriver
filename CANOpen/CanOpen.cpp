#include "CanOpen.h"
#include "utils.h"

int8_t MakeAbortCodePacket(uint32_t abortCode, CAN_OPEN_SDO_MSG *canOpenSdoMsg) 
{
	if(abortCode != CAN_OPEN_ABORT_CODE_NO_ERROR) {
		canOpenSdoMsg->cmd = SDO_ABORT;
		Uint32ToBytes(abortCode, canOpenSdoMsg->data);
		return 4;
	}
	else {
		canOpenSdoMsg->cmd = SDO_DOWNLOAD_RESPONSE;
		return 0;
	}
}
	

CanOpen::CanOpen(uint32_t vendorId, uint32_t productCode, uint32_t revisionNumber, uint32_t serialNumber)
: CanOpenCia402(vendorId, productCode, revisionNumber, serialNumber)
{
}

int8_t CanOpen::ProcessSdoCanOpen(uint32_t *id, uint8_t *data, uint32_t *len)
{
	int8_t ret = 4;
	uint32_t      nodeId; 
	uint32_t      cob_id; 
	CAN_OPEN_SDO_MSG canOpenSdoMsg;

	nodeId = (*id) & NODE_ID_MASK;//NODE__MASK;//pjg<>171025
	if(nodeId != _id)	return -1;
	
	cob_id = (*id) & COB_ID_MASK;

	if(cob_id == NMT_COB_ID)
	{
		//coe_nmtMsgHandler(id, data, len);
	}
	else if(cob_id == RSDO_COB_ID)
	{
		canOpenSdoMsg.cmd = data[0];
		canOpenSdoMsg.index = data[2] * 256 + data[1];
		canOpenSdoMsg.subIndex = data[3];
		memcpy(canOpenSdoMsg.data, &data[4], 4);	
		ret = ProcessSdo(&canOpenSdoMsg);
		*id = nodeId + TSDO_COB_ID;
		data[0] = (uint8_t)canOpenSdoMsg.cmd;
		data[1] = (uint8_t)(canOpenSdoMsg.index>>0);
		data[2] = (uint8_t)(canOpenSdoMsg.index>>8);
		data[3] = (uint8_t)canOpenSdoMsg.subIndex;	
		memcpy(&data[4], canOpenSdoMsg.data, ret);
		*len = ret + 4;		
	}
	else if((cob_id == RPDO1_COB_ID) 
		  || (cob_id == RPDO2_COB_ID) 
		  || (cob_id == RPDO3_COB_ID) 
		  || (cob_id == RPDO4_COB_ID))
	{
		//coe_pdoMsgHandler(id, data, len);
	}	
	
	return ret;
}

int8_t CanOpen::ProcessSdo(CAN_OPEN_SDO_MSG *canOpenSdoMsg)
{
	uint8_t ret;
	
	////////////////////////////////////////////////////////////////////////
	//	Buffer 0
	////////////////////////////////////////////////////////////////////////
	if((canOpenSdoMsg->index >= 0x5000) && (canOpenSdoMsg->index < 0x53E8)) {
		if(canOpenSdoMsg->subIndex == 0) {
			if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
				canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
				Int32ToBytes(GetBuf0(canOpenSdoMsg->index - 0x5000), canOpenSdoMsg->data);
				ret = 4;
			}	
			else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
		}
		else
			ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
		return ret;
	}
	////////////////////////////////////////////////////////////////////////
	//	Buffer 1
	////////////////////////////////////////////////////////////////////////
	else if((canOpenSdoMsg->index >= 0x5400) && (canOpenSdoMsg->index < 0x57E8)) {
		if(canOpenSdoMsg->subIndex == 0) {
			if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
				canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
				Int32ToBytes(GetBuf1(canOpenSdoMsg->index - 0x5400), canOpenSdoMsg->data);
				ret = 4;
			}	
			else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
		}
		else
			ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
		return ret;
	}

	switch(canOpenSdoMsg->index) {
		////////////////////////////////////////////////////////////////////////
		//	Control Word
		////////////////////////////////////////////////////////////////////////
		case CIA_402_CONTROL_WORD :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_controlWord, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetControlWord(BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
		
		////////////////////////////////////////////////////////////////////////
		//	Status Word
		////////////////////////////////////////////////////////////////////////
		case CIA_402_STATUS_WORD :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_statusWord, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Target Position
		////////////////////////////////////////////////////////////////////////
		case CIA_402_TARGET_POSITION :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_targetPosition, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetTargetPosition(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Actual Position
		////////////////////////////////////////////////////////////////////////
		case CIA_402_ACTUAL_POSITION :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_actualPosition, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Target Torque
		////////////////////////////////////////////////////////////////////////
		case CIA_402_TARGET_TORQUE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Int16ToBytes(_targetTorque, canOpenSdoMsg->data);
					ret = 2;
				}
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetTargetTorque(BytesToInt16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Target Offset
		////////////////////////////////////////////////////////////////////////
		case CIA_402_TORQUE_OFFSET :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Int16ToBytes(_torqueOffset, canOpenSdoMsg->data);
					ret = 2;
				}
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetTorqueOffset(BytesToInt16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Actual Torque
		////////////////////////////////////////////////////////////////////////
		case CIA_402_ACTUAL_TORQUE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Int16ToBytes(_actualTorque, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
	
		////////////////////////////////////////////////////////////////////////
		//	Target Current
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_TARGET_CURRENT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_targetCurrent, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetTargetCurrent(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
		
		////////////////////////////////////////////////////////////////////////
		//	Actual Current
		////////////////////////////////////////////////////////////////////////
		case CIA_402_ACTUAL_CURRENT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Int16ToBytes(_actualCurrent, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Target Velocity
		////////////////////////////////////////////////////////////////////////
		case CIA_402_TARGET_VELOCITY :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_targetVelocity, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetTargetVelocity(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Actual Velocity
		////////////////////////////////////////////////////////////////////////
		case CIA_402_ACTUAL_VELOCITY :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_actualVelocity, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Modes of operation
		////////////////////////////////////////////////////////////////////////
		case CIA_402_MODES_OF_OPERATION :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(_modesOfOperation, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetModesOfOperation(BytesToInt8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
		
		////////////////////////////////////////////////////////////////////////
		//	Modes of operation display
		////////////////////////////////////////////////////////////////////////
		case CIA_402_MODES_OF_OPERATION_DISPLAY :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(_modesOfOperationDisplay, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Position Following Error
		////////////////////////////////////////////////////////////////////////
		case CIA_402_POSITION_FOLLOWING_ERROR :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_positionFollowingError, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Position Demand Value
		////////////////////////////////////////////////////////////////////////
		case CIA_402_POSITION_DEMAND_VALUE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_demandPosition, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Velocity Demand Value
		////////////////////////////////////////////////////////////////////////
		case CIA_402_DEMAND_VELOCITY :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_demandVelocity, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Error Register
		////////////////////////////////////////////////////////////////////////
		case CIA_301_ERROR_REG :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(_errorReg, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Sync Manager Channel 2
		////////////////////////////////////////////////////////////////////////
		case CIA_301_SYNC_MANAGER_CHANNEL2 :
			if(canOpenSdoMsg->subIndex == CIA_301_SM2_COUNT) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(_syncMgrCh2NumberOfMappingObject, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetSyncMgrCh2NumberOfMappingObject(BytesToUint8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_301_SM2_PDO1) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(GetSyncMgrCh2MappingObject(canOpenSdoMsg->subIndex - 1), canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetSyncMgrCh2MappingObject(canOpenSdoMsg->subIndex - 1, BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Sync Manager Channel 3
		////////////////////////////////////////////////////////////////////////
		case CIA_301_SYNC_MANAGER_CHANNEL3 :
			if(canOpenSdoMsg->subIndex == CIA_301_SM3_COUNT) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(_syncMgrCh3NumberOfMappingObject, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetSyncMgrCh3NumberOfMappingObject(BytesToUint8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_301_SM3_PDO1) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(GetSyncMgrCh3MappingObject(canOpenSdoMsg->subIndex - 1), canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetSyncMgrCh3MappingObject(canOpenSdoMsg->subIndex - 1, BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
		
		////////////////////////////////////////////////////////////////////////
		//	Resistance
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_RESISTANCE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_resistance, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetResistance(BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Q-Axis Inductance
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_Q_AXIS_INDUCTANCE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_qAxisInductance, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetQAxisInductance(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	D-Axis Inductance
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_D_AXIS_INDUCTANCE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_dAxisInductance, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetQAxisInductance(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Torque Constant
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_TORQUE_CONSTANT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_torqueConstant, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetTorqueConstant(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Back-EMF Constant
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_BACK_EMF_CONSTANT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_backEmfConstant, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetBackEmfConstant(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	System Inertia
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_SYSTEM_INERTIA :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_systemInertia, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetSystemInertia(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Coulomb Friction
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_COULOMB_FRICTION :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_coulombFriction, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetCoulombFriction(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Viscos Friction
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_VISCOS_FRICTION :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_viscosFriction, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetViscosFriction(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Electric Angle Offset
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_ELECTRIC_ANGLE_OFFSET :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Int16ToBytes(_electricAngleOffset, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetElectricAngleOffset(BytesToInt16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
				
		////////////////////////////////////////////////////////////////////////
		//	Hall Sensor Pole Pair
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_MOTOR_PHASE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(_motorPhase, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetMotorPhase(BytesToInt8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Hall Sensor Pole Pair
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_HALL_SENSOR_POLE_PAIR :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(_polePairs, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetPolePair(BytesToInt8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	D-Axis Voltage
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_D_AXIS_VOLTAGE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_dAxisVoltage, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetDAxisVoltage(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Q-Axis Voltage
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_Q_AXIS_VOLTAGE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_qAxisVoltage, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetQAxisVoltage(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Hall Sensor Pattern
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_HALL_SENSOR_PATTERN :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(_hallSensorPattern, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	D-Axis Actual Current
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_D_AXIS_CURRENT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_dAxisActualCurrent, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Q-Axis Actual Current
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_Q_AXIS_CURRENT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_qAxisActualCurrent, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Temperature
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_TEMPERATURE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(_temperature, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
				
		////////////////////////////////////////////////////////////////////////
		//	Analog Inputs
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_ANALOG_INPUT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(4, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= 4) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(GetAnalogIntput(canOpenSdoMsg->subIndex - 1), canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Averaged Current
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_420_AVERAGED_CURRENT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_averagedCurrent, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Load Torque
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_LOAD_TORQUE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_loadTorque, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Load Torque Threshold
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_LOAD_TORQUE_THRESHOLD :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_loadTorqueThreshold, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetLoadTorqueThreshold(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Error Code
		////////////////////////////////////////////////////////////////////////
		case CIA_402_ERROR_CODE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_errorCode, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Following Error Window
		////////////////////////////////////////////////////////////////////////
		case CIA_402_FOLLOWING_ERROR_WINDOW :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_followingErrorWindow, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetFollowingErrorWindow(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Position Window
		////////////////////////////////////////////////////////////////////////
		case CIA_402_POSITION_WINDOW :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_positionWindow, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetPositionWindow(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Position Window Time
		////////////////////////////////////////////////////////////////////////
		case CIA_402_POSITION_WINDOW_TIME :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_positionWindowTime, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetPositionWindowTime(BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Velocity Window
		////////////////////////////////////////////////////////////////////////
		case CIA_402_VELOCITY_WINDOW :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_velocityWindow, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetVelocityWindow(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Velocity Window Time
		////////////////////////////////////////////////////////////////////////
		case CIA_402_VELOCITY_WINDOW_TIME :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_velocityWindowTime, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetVelocityWindowTime(BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Maximum Torque
		////////////////////////////////////////////////////////////////////////
		case CIA_402_MAXIMUM_TORQUE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Int16ToBytes(_maximumTorque, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetMaximumTorque(BytesToInt16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Maximum Current
		////////////////////////////////////////////////////////////////////////
		case CIA_402_MAXIMUM_CURRENT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_maximumCurrent, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetMaximumCurrent(BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
						
		////////////////////////////////////////////////////////////////////////
		//	Motor Rated Current
		////////////////////////////////////////////////////////////////////////
		case CIA_402_MOTOR_RATED_CURRENT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_ratedCurrent, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetRatedCurrent(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Motor Rated Torque
		////////////////////////////////////////////////////////////////////////
		case CIA_402_MOTOR_RATED_TORQUE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_ratedTorque, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetRatedTorque(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	DC Link Voltage
		////////////////////////////////////////////////////////////////////////
		case CIA_402_DC_LINK_VOLTAGE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_dcLinkVoltage, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Maximal Profile Velocity
		////////////////////////////////////////////////////////////////////////
		case CIA_402_MAXIMAL_PROFILE_VELOCITY :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_maxProfileVelocity, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetMaxProfileVelocity(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Maximum Motor Speed
		////////////////////////////////////////////////////////////////////////
		case CIA_402_MAXIMUM_MOTOR_SPEED :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_maxMotorSpeed, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetMaxMotorSpeed(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Profile Velocity
		////////////////////////////////////////////////////////////////////////
		case CIA_402_PROFILE_VELOCITY :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_profileVelocity, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetProfileVelocity(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Profile Acceleration
		////////////////////////////////////////////////////////////////////////
		case CIA_402_PROFILE_ACCELERATION :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_profileAcceleration, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetProfileAcceleration(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Profile Deceleration
		////////////////////////////////////////////////////////////////////////
		case CIA_402_PROFILE_DECELERATION :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_profileDeceleration, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetProfileDeceleration(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Quick Stop Deceleration
		////////////////////////////////////////////////////////////////////////
		case CIA_402_QUICK_STOP_DECELERATION :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_quickStopDeceleration, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetQuickStopDeceleration(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Maximum Acceleration
		////////////////////////////////////////////////////////////////////////
		case CIA_402_MAX_ACCELERATION :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_maxAcceleration, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetMaxAcceleration(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Maximum Deceleration
		////////////////////////////////////////////////////////////////////////
		case CIA_402_MAX_DECELERATION :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_maxDeceleration, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetMaxDeceleration(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Motion Profile Type
		////////////////////////////////////////////////////////////////////////
		case CIA_402_MOTION_PROFILE_TYPE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Int16ToBytes(_motionProfileType, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetMotionProfileType(BytesToInt16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	SW Poisition Limit
		////////////////////////////////////////////////////////////////////////
		case CIA_402_POSITION_SW_LIMIT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(2, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_402_MINIMAL_SW_POSITION_LIMIT) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_minSwPositionLimit, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetMinSwPositionLimit(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_402_MAXIMAL_SW_POSITION_LIMIT) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_maxSwPositionLimit, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetMaxSwPositionLimit(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Position Notation Index
		////////////////////////////////////////////////////////////////////////
		case CIA_402_POSITION_NOTATION_INDEX :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(_positionNotationIndex, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetPositionNotationIndex(BytesToInt8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Position Dimenstion Index
		////////////////////////////////////////////////////////////////////////
		case CIA_402_POSITION_DIMENSION_INDEX :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(_positionDimensionIndex, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetPositionDimensionIndex(BytesToUint8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Velocity Notation Index
		////////////////////////////////////////////////////////////////////////
		case CIA_402_VELOCITY_NOTATION_INDEX :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(_velocityNotationIndex, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetVelocityNotationIndex(BytesToInt8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Velocity Dimenstion Index
		////////////////////////////////////////////////////////////////////////
		case CIA_402_VELOCITY_DIMENSION_INDEX :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(_velocityDimensionIndex, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetVelocityDimensionIndex(BytesToUint8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Acceleration Notation Index
		////////////////////////////////////////////////////////////////////////
		case CIA_402_ACCELERATION_NOTATION_INDEX :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(_accelerationNotationIndex, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetAccelerationNotationIndex(BytesToInt8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Acceleration Dimenstion Index
		////////////////////////////////////////////////////////////////////////
		case CIA_402_ACCELERATION_DIMENSION_INDEX :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(_accelerationDimensionIndex, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetAccelerationDimensionIndex(BytesToUint8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

			
		////////////////////////////////////////////////////////////////////////
		//	Homing Method
		////////////////////////////////////////////////////////////////////////
		case CIA_402_HOMING_METHOD :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Int8ToBytes(_homingMethod, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetHomingMethod(BytesToInt8(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Homing Speed
		////////////////////////////////////////////////////////////////////////
		case CIA_402_HOMING_SPEED :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(2, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_402_SWITCH_SEARCH_SPEED) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_switchSearchVelocity, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetSwitchSearchVelocity(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_402_ZERO_SEARCH_SPEED) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_zeroSearchVelocity, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetZeroSearchVelocity(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Homing Acceleration
		////////////////////////////////////////////////////////////////////////
		case CIA_402_HOMING_ACCELERATION :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_homingAcceleration, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetHomingAcceleration(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Home Offset
		////////////////////////////////////////////////////////////////////////
		case CIA_402_HOME_OFFSET :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_homeOffset, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetHomeOffset(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Current Parameters
		////////////////////////////////////////////////////////////////////////
		case CIA_402_CURRENT_PARAMETER :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(2, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_402_CURRENT_P_GAIN) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_currentPGain, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetCurrentPGain(BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_402_CURRENT_I_GAIN) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_currentIGain, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetCurrentIGain(BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Velocity Parameter
		////////////////////////////////////////////////////////////////////////
		case CIA_402_VELOCITY_PARAMETER :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(2, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_402_VELOCITY_P_GAIN) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_velocityPGain, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetVelocityPGain(BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_402_VELOCITY_I_GAIN) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_velocityIGain, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetVelocityIGain(BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
	
		////////////////////////////////////////////////////////////////////////
		//	Position Parameter
		////////////////////////////////////////////////////////////////////////
		case CIA_402_POSITION_PARAMETER :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(1, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_402_POSITION_P_GAIN) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_positionPGain, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetPositionPGain(BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
		
		////////////////////////////////////////////////////////////////////////
		//	Digital Input
		////////////////////////////////////////////////////////////////////////
		case CIA_402_DIGITAL_INPUT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_digitalInput, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

		////////////////////////////////////////////////////////////////////////
		//	Digital Output
		////////////////////////////////////////////////////////////////////////
		case CIA_402_DIGITAL_OUTPUT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(2, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_402_DIGITAL_OUTPUT_PHYSICAL_OUTPUT) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_digitalOutput, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetDigitalOutput(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_402_DIGITAL_OUTPUT_OUTPUT_MASK) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint32ToBytes(_digitalOutputMask, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetDigitalOutputMask(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
	
			
		////////////////////////////////////////////////////////////////////////
		//	Position-Current-Position Mode Parameters
		////////////////////////////////////////////////////////////////////////
		case KITECH_CIA_402_PCP_MODE_PARAMETER :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(4, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == KITECH_CIA_402_PCP_MODE_HOME_POSITION) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_pcpModeHomePosition, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetPcpModeHomePosition(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == KITECH_CIA_402_PCP_MODE_TARGET_POSITION) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_pcpModeTargetPosition, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetPcpModeTargetPosition(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == KITECH_CIA_402_PCP_MODE_TARGET_CURRENT) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Int32ToBytes(_pcpModeTargetCurrent, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetPcpModeTargetCurrent(BytesToInt32(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == KITECH_CIA_402_PCP_MODE_CURRENT_MODE_DURATION) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_WORD;
					Uint16ToBytes(_pcpModeCurrentModeDuration, canOpenSdoMsg->data);
					ret = 2;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_WORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(SetPcpModeCurrentModeDuration(BytesToUint16(canOpenSdoMsg->data)), canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Device Type
		////////////////////////////////////////////////////////////////////////
		case CIA_301_DEVICE_TYPE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_deviceType, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Vendor ID
		////////////////////////////////////////////////////////////////////////
		case CIA_301_INDENTITY_OBJECT :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = 4;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_301_VENDOR_ID) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_vendorId, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_301_PRODUCT_CODE) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_productCode, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_301_REVISION_NUMBER) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_revisionNumber, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_301_SERIAL_NUMBER) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_serialNumber, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	RxPDO1 Mapping
		////////////////////////////////////////////////////////////////////////
		case CIA_301_RX_PDO_MAPPING1 :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = 3;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= CIA_301_PDO_MAPPING_OBJ3) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_rxPdo1MappingObject[canOpenSdoMsg->subIndex - 1], canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	RxPDO2 Mapping
		////////////////////////////////////////////////////////////////////////
		case CIA_301_RX_PDO_MAPPING2 :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = 3;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= CIA_301_PDO_MAPPING_OBJ4) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_rxPdo2MappingObject[canOpenSdoMsg->subIndex - 1], canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	RxPDO3 Mapping
		////////////////////////////////////////////////////////////////////////
		case CIA_301_RX_PDO_MAPPING3 :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = 3;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= CIA_301_PDO_MAPPING_OBJ3) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_rxPdo3MappingObject[canOpenSdoMsg->subIndex - 1], canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	RxPDO4 Mapping
		////////////////////////////////////////////////////////////////////////
		case CIA_301_RX_PDO_MAPPING4 :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = 3;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= CIA_301_PDO_MAPPING_OBJ3) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_rxPdo4MappingObject[canOpenSdoMsg->subIndex - 1], canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	RxPDO5 Mapping
		////////////////////////////////////////////////////////////////////////
		case CIA_301_RX_PDO_MAPPING5 :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = 3;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED))) 
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= CIA_301_PDO_MAPPING_OBJ6) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_rxPdo5MappingObject[canOpenSdoMsg->subIndex - 1], canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	TxPDO1 Mapping
		////////////////////////////////////////////////////////////////////////
		case CIA_301_TX_PDO_MAPPING1 :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = 3;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= CIA_301_PDO_MAPPING_OBJ3) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_txPdo1MappingObject[canOpenSdoMsg->subIndex - 1], canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
			
		////////////////////////////////////////////////////////////////////////
		//	TxPDO2 Mapping
		////////////////////////////////////////////////////////////////////////
		case CIA_301_TX_PDO_MAPPING2 :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = 3;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= CIA_301_PDO_MAPPING_OBJ4) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_txPdo2MappingObject[canOpenSdoMsg->subIndex - 1], canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	TxPDO3 Mapping
		////////////////////////////////////////////////////////////////////////
		case CIA_301_TX_PDO_MAPPING3 :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = 3;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= CIA_301_PDO_MAPPING_OBJ5) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_txPdo3MappingObject[canOpenSdoMsg->subIndex - 1], canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;

			
		////////////////////////////////////////////////////////////////////////
		//	TxPDO4 Mapping
		////////////////////////////////////////////////////////////////////////
		case CIA_301_TX_PDO_MAPPING4 :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = 3;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= CIA_301_PDO_MAPPING_OBJ5) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_txPdo4MappingObject[canOpenSdoMsg->subIndex - 1], canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;			
			
		////////////////////////////////////////////////////////////////////////
		//	TxPDO5 Mapping
		////////////////////////////////////////////////////////////////////////
		case CIA_301_TX_PDO_MAPPING5 :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = 3;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= CIA_301_PDO_MAPPING_OBJ6) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(_txPdo5MappingObject[canOpenSdoMsg->subIndex - 1], canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Sync Manager Communication Type
		////////////////////////////////////////////////////////////////////////
		case CIA_301_SYNC_MANAGER_COMMUNICATION_TYPE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = _numberOfSyncManagerChannels;
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex <= CIA_301_COMM_TYPE_SYNC_MANAGER_CHANNEL8) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					canOpenSdoMsg->data[0] = _commTypeSyncManagerChannel[canOpenSdoMsg->subIndex - 1];
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		////////////////////////////////////////////////////////////////////////
		//	Store Parameters
		////////////////////////////////////////////////////////////////////////
		case CIA_301_STORE :
			if(canOpenSdoMsg->subIndex == 0) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_BYTE;
					Uint8ToBytes(1, canOpenSdoMsg->data);
					ret = 1;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_BYTE)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED)))
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR, canOpenSdoMsg);
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else if(canOpenSdoMsg->subIndex == CIA_301_STORE_ALL_PARAMETERS) {
				if((canOpenSdoMsg->cmd & SDO_CMD_SPECIFIER) == SDO_UPLOAD_REQUEST) {
					canOpenSdoMsg->cmd = SDO_UPLOAD_RESPONSE | SDO_SIZE_DWORD;
					Uint32ToBytes(0x00000000, canOpenSdoMsg->data);
					ret = 4;
				}	
				else if((canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_DWORD)) || (canOpenSdoMsg->cmd == (SDO_DOWNLOAD_REQUEST | SDO_SIZE_UNDEFINED))) {
					SetControlWord(CIA_402_CONTROL_DISABLE_VOLTAGE);
					ret = MakeAbortCodePacket(StoreAllParameters(BytesToUint32(canOpenSdoMsg->data)), canOpenSdoMsg);
				}
				else
					ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_ACCESS_ERROR, canOpenSdoMsg);
			}
			else
				ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR, canOpenSdoMsg);
			break;
			
		default :
			ret = MakeAbortCodePacket(CAN_OPEN_ABORT_CODE_OBJECT_INDEX_ERROR, canOpenSdoMsg);
	}
	
	return ret;
}