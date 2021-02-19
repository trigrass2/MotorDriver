#pragma once

#include <stdint.h>

#define COB_ID_MASK			0x0780
#define NODE_ID_MASK		0x007F

#define NMT_COB_ID 	0x0000

#define TSDO_COB_ID 0x580
#define RSDO_COB_ID 0x600

#define TPDO1_COB_ID 0x180
#define RPDO1_COB_ID 0x200
#define TPDO2_COB_ID 0x280
#define RPDO2_COB_ID 0x300
#define TPDO3_COB_ID 0x380
#define RPDO3_COB_ID 0x400
#define TPDO4_COB_ID 0x480
#define RPDO4_COB_ID 0x500

#define TPDO1_MAP_INDEX 0x1A00
#define RPDO1_MAP_INDEX 0x1600
#define TPDO2_MAP_INDEX 0x1A01
#define RPDO2_MAP_INDEX 0x1601
#define TPDO3_MAP_INDEX 0x1A02
#define RPDO3_MAP_INDEX 0x1602
#define TPDO4_MAP_INDEX 0x1A03
#define RPDO4_MAP_INDEX 0x1603

#define TPDO1_COMM_INDEX 0x1800
#define RPDO1_COMM_INDEX 0x1400
#define TPDO2_COMM_INDEX 0x1801
#define RPDO2_COMM_INDEX 0x1401
#define TPDO3_COMM_INDEX 0x1802
#define RPDO3_COMM_INDEX 0x1402
#define TPDO4_COMM_INDEX 0x1803
#define RPDO4_COMM_INDEX 0x1403

//	Error Information
#define	CAN_OPEN_ERROR_CODE_NO_ERROR							0x00000000
#define	CAN_OPEN_ERROR_CODE_GENERIC								0x00001000
#define	CAN_OPEN_ERROR_CODE_OVER_CURRENT						0x00002310	//	Too high controller gains
#define	CAN_OPEN_ERROR_CODE_OVER_VOLTAGE						0x00003210	//	The power supply voltage is too high
#define	CAN_OPEN_ERROR_CODE_UNDER_VOLTAGE						0x00003220	//	The power supply voltage is too low
#define	CAN_OPEN_ERROR_CODE_OVER_TEMPERATURE					0x00004210	//	The temperature at the device power stage is too high
#define	CAN_OPEN_ERROR_CODE_HALL_SENSOR							0x0000FF01	//	The motor hall sensors report an impossible signal combination

#define	CAN_OPEN_ABORT_CODE_NO_ERROR							0x00000000	//	No Error
#define	CAN_OPEN_ABORT_CODE_OBJECT_INDEX_ERROR					0x06020000	//	The last read or write command had a wrong index
#define	CAN_OPEN_ABORT_CODE_OBJECT_SUB_INDEX_ERROR				0x06090011	//	The last read or write command had a wrong sub-index
#define	CAN_OPEN_ABORT_CODE_ACCESS_ERROR						0x06010000	//	Unsupported access to an object
#define	CAN_OPEN_ABORT_CODE_WRITE_ONLY_ERROR					0x06010001	//	Read command to a write only object
#define	CAN_OPEN_ABORT_CODE_READ_ONLY_ERROR						0x06010002	//	Write command to a read only object

#define	CAN_OPEN_ABORT_CODE_SERVICE_PARAMETER_ERROR				0x06070010	//	Data type does not match, length or service parameter does not match
#define	CAN_OPEN_ABORT_CODE_SERVICE_PARAMETER_TOO_HIGH_ERROR	0x06070012	//	Data type does not match, length or service parameter too high
#define	CAN_OPEN_ABORT_CODE_SERVICE_PARAMETER_TOO_LOW_ERROR		0x06070012	//	Data type does not match, length or service parameter too low

#define	CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR					0x06090030	//	Value rage of parameter exceeded
#define	CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR				0x06090031	//	Value of parameter written too high
#define	CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR					0x06090032	//	Value of parameter written too low

#define	CAN_OPEN_ABORT_CODE_MAXIMUM_LESS_MINIMUM_ERROR			0x06090036	//	Maximum value is less than minimum value

#define	CAN_OPEN_ABORT_CODE_GENERAL_ERROR						0x08000000	//	General Error
#define	CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR					0x08000021	//	Data cannot be transferred or stored to application because of local control
#define	CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE					0x08000022	//	Data cannot be transferred or stored to application because of the present device state

#define	CAN_OPEN_ABORT_CODE_ILLEGAL_COMMAND_ERROR				0x0F00FFBF	//	Command code is illegal(does not exist)
#define	CAN_OPEN_ABORT_CODE_SERVICE_MODE_ERROR					0x0F00FFBC	//	Device is not in service mode

//	Unit Information
#define	CAN_OPEN_NOTATION_INDEX_KILO							0x03	//	10^3
#define	CAN_OPEN_NOTATION_INDEX_HECTOR							0x02	//	10^2
#define	CAN_OPEN_NOTATION_INDEX_DECA							0x01	//	10^1
#define	CAN_OPEN_NOTATION_INDEX_ZERO							0x00	//	10^0
#define	CAN_OPEN_NOTATION_INDEX_DECI							0xFF	//	10^-1
#define	CAN_OPEN_NOTATION_INDEX_CENTI							0xFE	//	10^-2
#define	CAN_OPEN_NOTATION_INDEX_MILLI							0xFD	//	10^-3

#define	CAN_OPEN_DIMENSION_INDEX_A								0x04	//	Ampere
#define	CAN_OPEN_DIMENSION_INDEX_RPM_PER_SEC					0xA3	//	RPM/S
#define	CAN_OPEN_DIMENSION_INDEX_RPM							0xA4	//	Revolution Per Minute
#define	CAN_OPEN_DIMENSION_INDEX_STEPS							0xAC	//	Steps

#define	SDO_CMD_SPECIFIER										0xE0
#define	SDO_UPLOAD_REQUEST										0x40
#define	SDO_UPLOAD_RESPONSE										0x40
#define	SDO_DOWNLOAD_REQUEST									0x20
#define	SDO_DOWNLOAD_RESPONSE									0x60
#define	SDO_ABORT												0xC0

#define	SDO_SIZE_BYTE											0x0F
#define	SDO_SIZE_WORD											0x0B
#define	SDO_SIZE_DWORD											0x03
#define	SDO_SIZE_UNDEFINED										0x02

typedef struct _CAN_OPEN_SDO_MSG_ {
	uint8_t cmd;
	uint8_t subIndex;
	uint16_t index;
	uint8_t data[4];
} CAN_OPEN_SDO_MSG;
