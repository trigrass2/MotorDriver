#pragma once

#define 	HEXAR_CIA_402_DIGITAL_INPUT_CONFIG				0x2071			//	Unsigned 16Bit, RW
#define	KITECH_CIA_402_RESISTANCE							0x2800			//	Unsigned 16Bit, RW	m¥Ø
#define	KITECH_CIA_402_Q_AXIS_INDUCTANCE					0x2801			//	Unsigned 32Bit, RW	uH
#define	KITECH_CIA_402_D_AXIS_INDUCTANCE					0x2802			//	Unsigned 32Bit, RW	uH
#define	KITECH_CIA_402_TORQUE_CONSTANT						0x2803			//	Unsigned 32Bit,	RW	uNm
#define	KITECH_CIA_402_BACK_EMF_CONSTANT					0x2804			//	Unsigned 32Bit, RW	uV/(rad/s)
#define	KITECH_CIA_402_SYSTEM_INERTIA						0x2805			//	Unsigned 32Bit, RW	mgcm^2
#define	KITECH_CIA_402_COULOMB_FRICTION						0x2806			//	Unsigned 32Bit, RW	uNm
#define	KITECH_CIA_402_VISCOS_FRICTION						0x2807			//	Unsigned 32Bit, RW	uNm/(rad/s)
#define	KITECH_CIA_402_ELECTRIC_ANGLE_OFFSET				0x2808			//	Signed 16Bit,	RW	degree
#define	KITECH_CIA_402_MOTOR_PHASE							0x2809			//	Unsigned 8bit,	RW	0 ~ 5

#define	KITECH_CIA_402_POSITION_SENSOR_TYPE					0x2810			//	Unsigned 8Bit, RW
	#define	KITECH_CIA_402_INCREMENTAL_ENCODER							0x00
	#define	KITECH_CIA_402_INCREMENTAL_ENCODER_WITH_INDEX				0x01
	#define	KITECH_CIA_402_HALL_SENSOR									0x02
	#define	KITECH_CIA_402_INCREMENTAL_ENCODER_HALL_SENSOR				0x03
	#define	KITECH_CIA_402_INCREMENTAL_ENCODER_WITH_INDEX_HALL_SENSOR	0x04
	#define	KITECH_CIA_402_RESOLVER										0x10
	#define	KITECH_CIA_402_SSI											0x20
	#define	KITECH_CIA_402_BISS											0x30
	#define	KITECH_CIA_402_TAMAGAWA_17BIT								0x40

#define	KITECH_CIA_402_POSITION_SENSOR_POLARITY				0x2811			//	Unsigned 8Bit, RW
	#define	KITECH_CIA_402_INCREMENTAL_ENCODER_POLARITY_INVERTED	0x01
	#define	KITECH_CIA_402_HALL_SENSOR_POLARITY_INVERTED			0x02
	#define	KITECH_CIA_402_RESOLVER_INVERTED						0x04
	#define	KITECH_CIA_402_SSI_INVERTED								0x08
	#define	KITECH_CIA_402_BISS_INVERTED							0x10

#define	KITECH_CIA_402_HALL_SENSOR_POLE_PAIR				0x2812			//	Unsigned 8Bit, RW

#define	KITECH_CIA_402_D_AXIS_VOLTAGE						0x2910			//	Signed 32Bit,	RO	mV
#define	KITECH_CIA_402_Q_AXIS_VOLTAGE						0x2911			//	Signed 32Bit,	RO	mV
#define	KITECH_CIA_402_HALL_SENSOR_PATTERN					0x2915			//	Unsigned 8Bit,	RO

#define	KITECH_CIA_402_D_AXIS_CURRENT						0x2A00			//	Signed 32Bit,	RO	mA
#define	KITECH_CIA_402_Q_AXIS_CURRENT						0x2A01			//	Signed 32Bit,	RO	mA

#define	KITECH_CIA_402_TEMPERATURE							0x2B00			//	Unsigned 8Bit,	RO	¡ÆC
#define	KITECH_CIA_402_ANALOG_INPUT							0x2B01			//	Unsigned 8Bit, 	RO
	#define	KITECH_CIA_402_ANALOG_INPUT0						0x01			//	Unsigned 16Bit, 	RO
	#define	KITECH_CIA_402_ANALOG_INPUT1						0x02			//	Unsigned 16Bit, 	RO
	#define	KITECH_CIA_402_ANALOG_INPUT2						0x03			//	Unsigned 16Bit, 	RO
	#define	KITECH_CIA_402_ANALOG_INPUT3						0x04			//	Unsigned 16Bit, 	RO

#define	KITECH_CIA_402_TARGET_CURRENT						0x3000			//	Signed 32Bit,	RW	mA
#define	KITECH_CIA_420_AVERAGED_CURRENT						0x3001			//	Signed 32Bit,	RO	mA

#define	KITECH_CIA_402_LOAD_TORQUE							0x3010			//	Signed 32Bit, 	RO	mNm or mN
#define	KITECH_CIA_402_LOAD_TORQUE_THRESHOLD				0x3011			//	Signed 32Bit, 	RW	mNm or mN

#define	KITECH_CIA_402_VELCOITY_FOLLOWING_ERROR				0x3100			//	Unsigned 32Bit,	RW	RPM

#define	KITECH_CIA_402_VELOCITY_AUTO_TUNING_ACCELERATION	0x3600			//	Unsigned 8Bit,	RW	%
#define	KITECH_CIA_402_VELOCITY_AUTO_TUNING_VELOCITY		0x3601			//	Unsigned 8Bit,	RW	%
#define	KITECH_CIA_402_VELOCITY_AUTO_TUNING_POSITION		0x3602			//	Signed 32Bit,	RW	Count
#define	KITECH_CIA_402_VELOCITY_CONTROLLER_BANDWIDTH		0x3603			//	Unsigned 16Bit,	RW	rad/s


#define	MAX_CURRENT_CONTROLLER_BANDWIDTH					30000
#define	MIN_CURRENT_CONTROLLER_BANDWIDTH					600
#define	MAX_VELOCITY_CONTROLLER_BANDWIDTH					3000
#define	MIN_VELOCITY_CONTROLLER_BANDWIDTH					60

#define	KITECH_CIA_402_PCP_MODE_PARAMETER					0x4000
	#define	KITECH_CIA_402_PCP_MODE_HOME_POSITION				0x01		//	Signed 32Bit,	RW	pulse
	#define	KITECH_CIA_402_PCP_MODE_TARGET_POSITION				0x02		//	Signed 32Bit,	RW	pulse
	#define	KITECH_CIA_402_PCP_MODE_TARGET_CURRENT				0x03		//	Signed 32Bit,	RW	mA
	#define	KITECH_CIA_402_PCP_MODE_CURRENT_MODE_DURATION		0x04		//	Unsigned 16Bit,	RW	0.5ms

//pjg++180417
#define	HEXAR_CIA_402_CAN_ID_CHANGE							0x4100	//	Unsigned 8Bit,	RW	
//pjg++180709
#define	HEXAR_CIA_402_MOTOR_INFO_SEND_TYPE					0x4110	//	Unsigned 8Bit,	RW	

#define	PCP_MODE_STATUS_READY								0x00
#define	PCP_MODE_STATUS_TARGET_POSITION						0x01
#define	PCP_MODE_STATUS_TARGET_CURRENT						0x02
#define	PCP_MODE_STATUS_HOME_POSITION						0x03

#define	NEGATIVE_LIMIT_SENSOR								0x000000001
#define	POSITIVE_LIMIT_SENSOR								0x000000002
#define	HOMING_SENSOR										0x000000004
#define	BRAKE_SENSOR										0x000000008

#define	MAX_OVER_LOAD_COUNT									4000		//	0.5ms * 2000	-> 2s
#define	HOMING_OVER_LOAD_COUNT								1000		//	0.5ms * 1000	-> 0.5s

typedef struct _MOTOR_PROPERTY_ {
	uint32_t motorId;
	uint8_t id;
	
	uint16_t motorType;
	uint16_t resistance;
	uint32_t dAxisInductance;
	uint32_t qAxisInductance;
	uint32_t torqueConstant;
	uint32_t backEmfConstant;
	uint32_t systemInertia;
	uint32_t coulombFriction;
	uint32_t viscosFriction;
	int16_t elecAngleOffset;
	uint8_t motorPhase;
	
	uint32_t ratedCurrent;
	uint16_t maxCurrent;
	uint32_t ratedTorque;
	uint32_t maxMotorSpeed;
	
	uint8_t positionSensorType;
	uint8_t positionSensorPolarity;
	uint8_t polePairs;
	uint32_t positionSensorIncrement;
	
	uint16_t dAxisCurrentPGain;
	uint16_t dAixsCurrentIGain;
	uint16_t qAxisCurrentPGain;
	uint16_t qAxisCurrentIGain;
	uint16_t velocityPGain;
	uint16_t velocityIGain;
	uint16_t positionPGain;
	
	int16_t motionProfileType;
	uint32_t maxProfileVelocity;
	uint32_t profileVelocity;
	uint32_t profileAcceleration;
	uint32_t profileDeceleration;
	uint32_t quickStopDeceleration;
	uint32_t maxAcceleration;
	uint32_t maxDeceleration;
	
	int32_t minSwPositionLimit;
	int32_t maxSwPositionLimit;
	
	int8_t homingMethod;
	uint32_t switchSearchSpeed;
	uint32_t zeroSearchSpeed;
	uint32_t homingAcceleration;
	int32_t homeOffset;
} MOTOR_PROPERTY;

typedef struct _MOTOR_PROPERTY_EX_ {
	uint8_t header[2]; //hx
	uint16_t ver; // 01.00 ~ 99.FF
	MOTOR_PROPERTY motorProperty;
	uint32_t digitalInputMask;
	uint32_t digitalInputPolarity;
}MOTOR_PROPERTY_EX;

