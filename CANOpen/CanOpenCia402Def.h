#pragma once

#define	CIA_402_ERROR_CODE									0x603F			//	Unsinged 16bit, RO
#define	CIA_402_ERROR_CODE_GENERIC_ERROR					0x1000
#define	CIA_402_ERROR_CODE_OVER_CURRENT_ERROR				0x2310
#define	CIA_402_ERROR_CODE_OVER_VOLTAGE_ERROR				0x3210
#define	CIA_402_ERROR_CODE_UNDER_VOLTAGE_ERROR				0x3220
#define	CIA_402_ERROR_CODE_OVER_TEMPERATURE_ERROR			0x4210
#define	CIA_402_ERROR_CODE_CURRENT_DETECTION_ERROR			0x5210
#define	CIA_402_ERROR_CODE_SERIAL_ENCODER_ERROR				0x7300
#define	CIA_402_ERROR_CODE_ENCODER_DISCONNECTION_ERROR		0x7305
#define	CIA_402_ERROR_CODE_OVER_SPEED_ERROR					0x8400
#define	CIA_402_ERROR_CODE_FOLLOWING_ERROR					0x8611
#define	CIA_402_ERROR_CODE_HALL_SENSOR_ERROR				0xFF01
#define	CIA_402_ERROR_CODE_OVER_LOAD_ERROR					0xFF02
#define	CIA_402_ERROR_CODE_CURRENT_AUTO_TUNING_ERROR		0xFF10
#define	CIA_402_ERROR_CODE_VELOCITY_AUTO_TUNING_ERROR		0xFF11


#define	CIA_402_CONTROL_WORD								0x6040			//	Unsigned 16bit, RW
	#define	CIA_402_SWITCH_ON									0x0001
	#define	CIA_402_ENABLE_VOLTAGE								0x0002
	#define	CIA_402_QUICK_STOP									0x0004
	#define	CIA_402_ENABLE_OPERATION							0x0008
	#define	CIA_402_FAULT_RESET									0x0080

	#define	CIA_402_CONTROL_MODE_MASK							0x0370
		#define	CIA_402_CONTROL_HOMING_START						0x0010		//	Homing Mode
		#define	CIA_402_CONTROL_NEW_SETPOINT						0x0010		//	Profile Position Mode
		#define	CIA_402_CONTROL_ENABLE_RAMP							0x0010		//	Velocity L
		#define	CIA_402_CONTROL_ENABLE_IP_MODE						0x0010		//	Interpolated Position Mode
		#define	CIA_402_CONTROL_PCP_MODE_START						0x0010		//	Position-Current-Position Mode
		#define	CIA_402_CONTROL_CHANGE_SET_IMMEDIATELY				0x0020		//	Profile Position Mode
		#define	CIA_402_CONTROL_UNLOCK_RAMP							0x0020		//	Velocity L
		#define	CIA_402_CONTROL_RELATIVE_BIT						0x0040		//	Profile Position Mode
		#define	CIA_402_CONTROL_REFERENCE_RAMP						0x0040		//	Velocity L
		#define	CIA_402_CONTROL_HALT_BIT							0x0100		//	Homing, 
		#define	CIA_402_CONTROL_CHANGE_ON_SETPOINT					0x0200		//	Profile Position Mode

	#define	CIA_402_CONTROL_COMMAND_MASK						0x008F
		#define	CIA_402_CONTROL_SHUTDONW							0x0006
		#define	CIA_402_CONTROL_SHUTDOWN_MASK						0x0087
		#define	CIA_402_CONTROL_SWITCH_ON							0x0007
		#define	CIA_402_CONTROL_SWITCH_ON_MASK						0x008F
		#define	CIA_402_CONTROL_DISABLE_VOLTAGE						0x0000
		#define	CIA_402_CONTROL_DISABLE_VOLTAGE_MASK				0x0082
		#define	CIA_402_CONTROL_QUICK_STOP							0x0002
		#define	CIA_402_CONTROL_QUICK_STOP_MASK						0x0086
		#define	CIA_402_CONTROL_DISABLE_OPERATION					0x0007
		#define	CIA_402_CONTROL_DISABLE_OPERATION_MASK				0x008F
		#define	CIA_402_CONTROL_ENABLE_OPERATION					0x000F
		#define	CIA_402_CONTROL_ENABLE_OPERATION_MASK				0x008F
		#define	CIA_402_CONTROL_FAULT_RESET							0x0080
		#define	CIA_402_CONTROL_FAULT_RESET_MASK					0x0080
		#define	CIA_402_CONTROL_SWITCH_ON_ENABLE					0x000F
		#define	CIA_402_CONTROL_SWITCH_ON_ENABLE_MASK				0x008F

	#define	CIA_402_CONTROL_START_AUTO_TUNING						0x8000

#define	CIA_402_STATUS_WORD									0x6041			//	Unsigned 16bit,	RO
	#define	CIA_402_READY_TO_SWITCH_ON							0x0001
	#define	CIA_402_SWITCHED_ON									0x0002
	#define	CIA_402_OPERATION_ENABLE							0x0004
	#define	CIA_402_FAULT										0x0008
	#define	CIA_402_VOLTAGE_ENABLED								0x0010
	#define	CIA_402_IS_QUICK_STOP								0x0020
	#define	CIA_402_SWITCH_ON_DISABLE							0x0040
	#define	CIA_402_WARNING										0x0080
	#define	CIA_402_STOP_AND_POSITION_CONTROL					0x0100		//	KITECH
	#define	CIA_402_REMOTE										0x0200
	#define	CIA_402_TARGET_REACHED								0x0400
	#define	CIA_402_INTERNAL_LIMIT_ACTIVE						0x0800
	#define	CIA_402_HOMING_ATTAINED								0x1000
	#define	CIA_402_FOLLOWING_ERROR								0x2000
	#define	CIA_402_HOMING_ERROR								0x2000
	#define	CIA_402_PCP_MODE_ERROR								0x2000
	#define CIA_402_SUCCESS_AUTO_TUNING							0x8000

	#define	CIA_402_STATUS_MODE_MASK							0x3000
		#define	CIA_402_STATUS_HOMING_ATTAINED						0x1000		//	Homing Mode
		#define	CIA_402_STATUS_SPEED								0x1000		//	Profile Velocity Mode
		#define	CIA_402_STATUS_SET_POINT_ACK						0x1000		//	Profile Position Mode
		#define	CIA_402_STATUS_IP_MODE_ACK							0x1000		//	Interpolated Position Mode
		#define	CIA_402_STATUS_TARGET_POSITION_FOLLOWED				0x1000		//	Cyclic Sync Position Mode
		#define	CIA_402_STATUS_TARGET_VELOCITY_FOLLOWED				0x1000		//	Cyclic Sync Velocity Mode
		#define	CIA_402_STATUS_TARGET_TORQUE_FOLLOWED				0x1000		//	Cyclic Sync Torque Mode
		#define	CIA_402_STATUS_HOMING_ERROR							0x2000		//	Homing Error
		#define	CIA_402_STATUS_MAX_SLIPPAGE_ERROR					0x2000		//	Profile Velocity Mode
		#define	CIA_402_STATUS_FOLLOWING_ERROR						0x2000		//	Profile Position & CSP Mode
		#define CIA_402_STATUS_LOAD_TORQUE_THRESHOLD_REACHED		0x4000		//	HM, VM, PVM, PM, PPM, IPM, CSP, CSV

	#define	CIA_402_STATUS_STATE_MASK								0x006F
		#define	CIA_402_STATUS_BOOTUP								0x0000			//	Bootup
		#define	CIA_402_STATUS_NOT_READY_TO_SWITCH_ON				0x0000			//	The current offset will be measurend. The drive function is disabled.
		#define	CIA_402_STATUS_SWITCH_ON_DISABLED					0x0040			//	The drive initialization is complete. The driver parameters may be changed. The driver function is disabled.
		#define	CIA_402_STATUS_READY_TO_SWITCH_ON					0x0021			//	The drive parameters may be changed. The drive function is disabled.
		#define	CIA_402_STATUS_SWITCH_ON							0x0023			//	The drive function is disabled.
		#define	CIA_402_STATUS_OPERATION_ENABLED					0x0027			//	No faults have been detected. The drive function is enabled and power is applied to motor.
		#define	CIA_402_STATUS_QUICK_STOP_ACTIVE					0x0007			//	The quick stop function is being executed. The drive function is enabled and power is applied to motor.
		#define	CIA_402_STATUS_FAULT_REACTION_ACTIVE_ENABLED		0x000F			//	A fault has occurred in the drive. The quick stop function is being executed. The drive function is enabled and power is applied to motor
		#define	CIA_402_STATUS_FAULT								0x0008			//	A fault has occurred in the drive. The drive parameters may be changed. The drive function is disabled.

#define	CIA_402_MODES_OF_OPERATION							0x6060			//	Signed 8bit,	RW
	#define	CIA_402_CYCLIC_SYNC_TORQUE_MODE						10
	#define	CIA_402_CYCLIC_SYNC_VELOCITY_MODE					9
	#define	CIA_402_CYCLIC_SYNC_POSITION_MODE					8
	#define	CIA_402_INTERPOLATED_POSITION_MODE					7
	#define	CIA_402_HOMING_MODE									6
	#define	CIA_402_PROFILE_VELOCITY_MODE						3
	#define	CIA_402_PROFILE_POSITION_MODE						1
	#define	CIA_402_VOLTAGE_MODE								-1
	#define	CIA_402_CURRENT_MODE								-2
	#define	CIA_402_TORQUE_MODE									-3
	#define	CIA_402_VELOCITY_MODE								-4
	#define	CIA_402_POSITION_MODE								-5
	#define	CIA_402_CURRENT_AUTO_TUNE_MODE						-6
	#define	CIA_402_VELOCITY_AUTO_TUNE_MODE						-7
	#define	CIA_402_POSITION_AUTO_TUNE_MODE						-8
	#define	CIA_402_POSITION_CURRENT_POSITION_MODE				-10

	#define	CIA_402_VOLTAGE_AUTO_TUNE_MODE						-100

#define	CIA_402_MODES_OF_OPERATION_DISPLAY					0x6061			//	Signed 8bit,	RW
#define	CIA_402_POSITION_DEMAND_VALUE						0x6062			//	Signed 32Bit,	RO	pulse

#define	CIA_402_ACTUAL_POSITION								0x6064			//	Signed 32Bit,	RO	Unit	(-2147483648 ~ 2147483647)
#define	CIA_402_FOLLOWING_ERROR_WINDOW						0x6065			//	Unsinged 32Bit,	RW	pulse	(1 ~ 2147483647)

#define	CIA_402_POSITION_WINDOW								0x6067			//	Unsigned 32Bit,	RW	pulse	(0 ~ 4294967295)
#define	CIA_402_POSITION_WINDOW_TIME						0x6068			//	Unsigned 16Bit,	RW	msec	(0 ~ 65535)

#define	CIA_402_DEMAND_VELOCITY								0x606B			//	Signed 32Bit,	RO	RPM
#define	CIA_402_ACTUAL_VELOCITY								0x606C			//	Signed 32Bit,	RO	RPM
#define	CIA_402_VELOCITY_WINDOW								0x606D			//	Unsigned 32Bit,	RW	RPM		(0 ~ 4294967295)
#define	CIA_402_VELOCITY_WINDOW_TIME						0x606E			//	Unsigned 16Bit,	RW	msec	(0 ~ 65535)

#define	CIA_402_TARGET_TORQUE								0x6071			//	Signed 16Bit,	RW	0.1%	(-Maximum Torque ~ Maximum Torque)
#define	CIA_402_MAXIMUM_TORQUE								0x6072			//	Signed 16Bit,	RW	0.1%	(-2000 ~ 2000)
#define	CIA_402_MAXIMUM_CURRENT								0x6073			//	Unsigned 16Bit, RW	mA		(0 ~ 65535)

#define	CIA_402_MOTOR_RATED_CURRENT							0x6075			//	Unsinged 32Bit,	RW	mA		(1 ~ 4294967295)
#define	CIA_402_MOTOR_RATED_TORQUE							0x6076			//	Unsinged 32Bit,	RW	mNm		(1 ~ 4294967295)
#define	CIA_402_ACTUAL_TORQUE								0x6077			//	Signed 16Bit,	RO	mNm		(-32768 ~ 32767)
#define	CIA_402_ACTUAL_CURRENT								0x6078			//	Signed 16Bit,	RO	mA		(-32768 ~ 32767)
#define	CIA_402_DC_LINK_VOLTAGE								0x6079			//	Unsigned 32Bit,	RO	mV		(0 ~ 4294967295)

#define	CIA_402_TARGET_POSITION								0x607A			//	Signed 32Bit,	RW	Unit	(-2147483648 ~ 2147483647)
#define	CIA_402_POSITION_RANGE_LIMIT						0x607B
	#define	CIA_402_MINIMAL_POSITION_RANGE_LIMIT				0x01			//	Signed 32Bit,	RW	Unit	(-2147483648 ~ 2147483647)
	#define	CIA_402_MAXIMAL_POSITION_RANGE_LIMIT				0x02			//	Signed 32Bit,	RW	Unit	(-2147483648 ~ 2147483647)
#define	CIA_402_HOME_OFFSET									0x607C			//	Signed 32Bit,	RW	Count	(-2147483648 ~ 2147483647)

#define	CIA_402_POSITION_SW_LIMIT							0x607D
	#define	CIA_402_MINIMAL_SW_POSITION_LIMIT					0x01			//	Signed 32Bit,	RW	Unit	(-2147483648 ~ 2147483647)
	#define	CIA_402_MAXIMAL_SW_POSITION_LIMIT					0x02			//	Signed 32Bit,	RW	Unit	(-2147483648 ~ 2147483647)
#define	CIA_402_MAXIMAL_PROFILE_VELOCITY					0x607F			//	Unsigned 32Bit,	RW	RPM		(1 ~ 25000)

#define	CIA_402_MAXIMUM_MOTOR_SPEED							0x6080			//	Unsigned 32Bit, RW	RPM		(1 ~ 25000)
#define	CIA_402_PROFILE_VELOCITY							0x6081			//	Unsigned 32Bit, RW	RPM		(1 ~ 25000)
#define	CIA_402_PROFILE_ACCELERATION						0x6083			//	Unsigned 32Bit, RW	RPM/s	(1 ~ 100000000)
#define	CIA_402_PROFILE_DECELERATION						0x6084			//	Unsigned 32Bit, RW	RPS/s	(1 ~ 100000000)
#define	CIA_402_QUICK_STOP_DECELERATION						0x6085			//	Unsigned 32Bit, RW	RPM/s	(1 ~ 100000000)
#define	CIA_402_MOTION_PROFILE_TYPE							0x6086			//	Signed 16Bit,	RW
	#define	CIA_402_TRAPEZOIDAL_PROFILE							0x0000
	#define	CIA_402_SINUSOIDAL_PROFILE							0x0001

#define	CIA_402_POSITION_NOTATION_INDEX						0x6089			//	Signed 8Bit,	RW	
#define	CIA_402_POSITION_DIMENSION_INDEX					0x608A			//	Unsigned 8Bit,	RW
#define	CIA_402_VELOCITY_NOTATION_INDEX						0x608B			//	Signed 8Bit,	RW	
#define	CIA_402_VELOCITY_DIMENSION_INDEX					0x608C			//	Unsigned 8Bit,	RW
#define	CIA_402_ACCELERATION_NOTATION_INDEX					0x608D			//	Signed 8Bit,	RW
#define	CIA_402_ACCELERATION_DIMENSION_INDEX				0x608E			//	Unsigned 8Bit,	RW
#define	CIA_402_POSITION_ENCODER_RESOLUTION					0x608F			//	Unsigned 8Bit,	RO
	#define	CIA_402_POSITION_ENCODER_INCREMENT					0x01			//	Unsigned 32Bit,	RW
	#define	CIA_402_POSITION_ENCODER_MOTOR_REVOLUTION			0x02			//	Unsigned 32Bit, RW

#define	CIA_402_NOTATION_INDEX_GIGA							9
#define	CIA_402_NOTATION_INDEX_MEGA							6
#define	CIA_402_NOTATION_INDEX_KILO							3
#define	CIA_402_NOTATION_INDEX_HECTO						2
#define	CIA_402_NOTATION_INDEX_DECA							1
#define	CIA_402_NOTATION_INDEX_ZERO							0
#define	CIA_402_NOTATION_INDEX_DECI							-1
#define	CIA_402_NOTATION_INDEX_CENTI						-2
#define	CIA_402_NOTATION_INDEX_MILLI						-3
#define	CIA_402_NOTATION_INDEX_MICRO						-6
#define	CIA_402_NOTATION_INDEX_NANO							-9

#define	CIA_402_DIMENSION_INDEX_RPS							0xA3			//	Revolution/sec
#define	CIA_402_DIMENSION_INDEX_RPM							0xA4			//	Revolution/min
#define	CIA_402_DIMENSION_INDEX_STEPS						0xAC			//	Steps
#define	CIA_402_DIMENSION_INDEX_SPR							0xAD			//	Steps/Revolution


#define	CIA_402_HOMING_METHOD								0x6098			//	Signed 8Bit,	RW
	#define	CIA_402_HOMING_METHOD_ACTUAL_POSITION							37
	#define	CIA_402_HOMING_METHOD_INDEX_POSITIVE_SPEED						34
	#define	CIA_402_HOMING_METHOD_INDEX_NEGATIVE_SPEED						33
	#define	CIA_402_HOMING_METHOD_HOME_SWITCH_NEGATIVE_SPEED				27
	#define	CIA_402_HOMING_METHOD_HOME_SWITCH_POSITIVE_SPEED				23
	#define	CIA_402_HOMING_METHOD_POSITIVE_LIMIT_SWITCH						18
	#define	CIA_402_HOMING_METHOD_NEGATIVE_LIMIT_SWITCH						17
	#define	CIA_402_HOMING_METHOD_HOME_SWITCH_NEGATIVE_SPEED_INDEX			11
	#define	CIA_402_HOMING_METHOD_HOME_SWITCH_POSITIVE_SPEED_INDEX			7
	#define	CIA_402_HOMING_METHOD_POSITIVE_LIMIT_SWITCH_INDEX				2
	#define	CIA_402_HOMING_METHOD_NEGATIVE_LIMIT_SWITCH_INDEX				1
	#define	CIA_402_HOMING_METHOD_NONE										0
	#define	CIA_402_HOMING_METHOD_CURRENT_THRESHOLD_POSITIVE_SPEED_INDEX	-1
	#define	CIA_402_HOMING_METHOD_CURRENT_THRESHOLD_NEGATIVE_SPPED_INDEX	-2
	#define	CIA_402_HOMING_METHOD_CURRENT_THRESHOLD_POSITIVE_SPEED			-3
	#define	CIA_402_HOMING_METHOD_CURRENT_THRESHOLD_NEGATIVE_SPEED			-4

	#define	HOMING_STATUS_INIT									0x00		//	Velocity Control Mode, Target Velocity = 0
	#define	HOMING_STATUS_READY									0x01		//	Position Control Mode, Target Position = Actual Position
	#define	HOMING_STATUS_START									0x02		//	Velocity Control Mode
	#define	HOMING_STATUS_ZERO_SEARCH							0x03		//	Velocity Control Mode
	#define	HOMING_STATUS_SWITCH_SEARCH							0x04		//	Velocity Control Mode
	#define	HOMING_STATUS_SEARCH_COMPLETE						0x05		//	Position Control Mode
	#define	HOMING_STATUS_GO_TO_HOME_POSITION					0x06		//	Position Control Mode
	#define	HOMING_STATUS_COMPLETE								0x07		//	Position Control Mode

#define	CIA_402_HOMING_SPEED								0x6099			//	Unsigned 8Bit,	RO
	#define	CIA_402_SWITCH_SEARCH_SPEED							0x01			//	Unsigned 32Bit, RW	RPM			
	#define	CIA_402_ZERO_SEARCH_SPEED							0x02			//	Unsigned 32Bit, RW	RPM
#define	CIA_402_HOMING_ACCELERATION							0x609A			//	Unsigned 32Bit,	RW	RPM/s	(1 ~ 4294967295)

#define	CIA_402_POSITION_OFFSET								0x60B0			//	Signed 32Bit,	RW	Count	(-2147483648 ~ 2147483647)
#define	CIA_402_VELOCITY_OFFSET								0x60B1			//	Signed 32Bit,	RW	RPM		(-Max Motor Speed ~ Max Motor Speed)
#define	CIA_402_TORQUE_OFFSET								0x60B2			//	Signed 16Bit,	RW	0.1%	(-1000 ~ 1000)

#define	CIA_402_MAX_ACCELERATION							0x60C5			//	Unsigned 32Bit, RW	RPM/s	(1 ~ 4294967295)
#define	CIA_402_MAX_DECELERATION							0x60C6			//	Unsigned 32Bit,	RW	RPM/s	(1 ~ 4294967295)


#define	CIA_402_POSITION_FOLLOWING_ERROR					0x60F4				//	Signed 32Bit	RW	Count	(-2147483648 ~ 2147483647)

#define	CIA_402_CURRENT_PARAMETER							0x60F6
	#define	CIA_402_CURRENT_P_GAIN								0x01			//	Unsigned 16Bit,	RW	10^-3	(0 ~ 65535)
	#define	CIA_402_CURRENT_I_GAIN								0x02			//	Unsigned 16Bit,	RW	10^0	(0 ~ 65535)

#define	CIA_402_VELOCITY_PARAMETER							0x60F9
	#define	CIA_402_VELOCITY_P_GAIN								0x01			//	Unsigned 16Bit,	RW	10^-4	(0 ~ 65535)
	#define	CIA_402_VELOCITY_I_GAIN								0x02			//	Unsigned 16Bit,	RW	10^-4	(0 ~ 65535)

#define	CIA_402_POSITION_PARAMETER							0x60FB
	#define	CIA_402_POSITION_P_GAIN								0x01			//	Unsigned 16Bit,	RW	10^-2	(0 ~ 65535)

#define	CIA_402_DIGITAL_INPUT								0x60FD			//	Unsigned 32Bit,	RO	
	#define	CIA_402_NEGATIVE_LIMIT_BIT							0x00000001
	#define	CIA_402_POSITIVE_LIMIT_BIT							0x00000002
	#define	CIA_402_HOME_BIT									0x00000004
	#define	CAI_402_EMERGENCY_BIT								0x00000008

#define	CIA_402_DIGITAL_OUTPUT								0x60FE			//	Unsigned 8Bit,	RO
	#define	CIA_402_DIGITAL_OUTPUT_PHYSICAL_OUTPUT				0x01			//	Unsigned 32Bit,	RW
	#define	CIA_402_DIGITAL_OUTPUT_OUTPUT_MASK					0x02			//	Unsigned 32Bit, RW

#define	CIA_402_TARGET_VELOCITY								0x60FF			//	Signed 32Bit,	RW	RPM		(-2147483648 ~ 2147483647)

#define	CIA_402_MOTOR_TYPE									0x6402			//	Unsigned 16Bit,	RW
	#define	CIA_402_DC_MOTOR									0x0001			//	Brushed DC Motor
	#define	CIA_402_LINEAR_DC_MOTOR								0x0002			//	Linear DC Motor
	#define	CIA_402_PMSM_MOTOR									0x000A			//	Sinusoidal PM BL Motor
	#define	CIA_402_BLDC_MOTOR									0x000B			//	Trapezoidal PM BL Motor
