#pragma once

#include <stdint.h>

#include "CanOpenCia301.h"
#include "CanOpenCia402Def.h"
#include "CanOpenCia402KitechDef.h"

#define	BUF_SIZE	1000

#define	STATUS_WORD_STATE	(_statusWord & CIA_402_STATUS_STATE_MASK)
#define	STATUS_WORD_MODE	(_statusWord & CIA_402_STATUS_MODE_MASK)

enum eMOTRO_INFO_SEND_TYPE_ORDER { //pjg++180710
	//position
	MISTO_CIA_402_TARGET_POSITION				= 0x00000001,
	MISTO_CIA_402_ACTUAL_POSITION				= 0x00000002,
	MISTO_CIA_402_POSITION_DEMAND_VALUE		= 0x00000004,
	MISTO_CIA_402_POSITION_FOLLOWING_ERROR		= 0x00000008,
	MISTO_CIA_402_RESERVED_POSITION				= 0x00000010,
	//torque
	MISTO_CIA_402_TARGET_TORQUE					= 0x00000020,
	MISTO_CIA_402_ACTUAL_TORQUE					= 0x00000040,
	MISTO_KITECH_CIA_402_LOAD_TORQUE			= 0x00000080,
	MISTO_CIA_402_RESERVED_TORQUE				= 0x00000100,
	//velocity
	MISTO_CIA_402_TARGET_VELOCITY				= 0x00000200,
	MISTO_CIA_402_ACTUAL_VELOCITY				= 0x00000400,
	MISTO_CIA_402_DEMAND_VELOCITY				= 0x00000800,
	MISTO_CIA_402_RESERVED_VELOCITY				= 0x00001000,
	//current
	MISTO_KITECH_CIA_402_TARGET_CURRENT			= 0x00002000,
	MISTO_CIA_402_ACTUAL_CURRENT				= 0x00004000,
	MISTO_KITECH_CIA_420_AVERAGED_CURRENT		= 0x00008000,
	MISTO_KITECH_CIA_402_D_AXIS_CURRENT			= 0x00010000,
	MISTO_KITECH_CIA_402_Q_AXIS_CURRENT			= 0x00020000,
	MISTO_CIA_402_RESERVED_CURRENT				= 0x00040000,
	//etc
	MISTO_CIA_402_STATUS_WORD					= 0x01000000,
	MISTO_CAN_OPEN_DC_LINK_VOLTAGE				= 0x02000000,
	MISTO_KITECH_CIA_402_TEMPERATURE			= 0x04000000,
	MISTO_CIA_402_DIGITAL_INPUT					= 0x08000000,
	MISTO_KITECH_CIA_402_ANALOG_INPUT			= 0x10000000,
	MISTO_MAX
};


typedef int8_t (*OnCia402StateMachineChangnedFunc)(void);

typedef int8_t (*SetTargetVoltageFunc)(int32_t dAxisVoltage, int32_t qAxisVoltage);
typedef int8_t (*SetTargetCurrentFunc)(int32_t targetCurrent);
typedef int8_t (*SetTargetTorqueFunc)(int16_t targetTorque);
typedef int8_t (*SetTargetVelocityFunc)(int32_t targetVelocity);
typedef int8_t (*SetTargetProfileVelocityFunc)(int32_t targetProfileVelocity, uint32_t maxProfileVelocity, uint32_t acceleration, uint32_t deceleration);
typedef int8_t (*SetTargetPositionFunc)(int32_t targetPosition);
typedef int8_t (*SetTargetProfilePositionFunc)(int32_t targetProfilePosition, uint32_t profileVelocity, uint32_t acceleration, uint32_t deceleration);
typedef int8_t (*SetFaultFunc)(void);
typedef int8_t (*SetPositionEncoderSensorValueFunc)(int32_t positionEncoderSensorValue);

typedef void (*RunBrakeFunc)(uint16_t runTime);

////////////////////////////////////////////////////////////////////////////////
//
//	Unit
//
//	Voltage							mV
//	Current							mA
//	Rated Torque					mNm
//	Target/Acutal Torque			0.1%
//	Torque Constant					uNm
//	Acceleration					RPM/s
//	Velocity						RPM
//	Position						Count
//	WindowTime						msec
//	Temperature						¡ÆC
//
////////////////////////////////////////////////////////////////////////////////


class CanOpenCia402 : public CanOpenCia301
{
public:
	CanOpenCia402(uint32_t vendorId, uint32_t productCode, uint32_t revisionNumber, uint32_t serialNumber);
	
	//	Hardware Limitation
	uint32_t _maxContCurrent;
	uint16_t _maxOutputCurrent;
	int16_t _nOverLoad;
	
	//	Control Variables
	uint16_t _controlWord;					uint32_t SetControlWord(uint16_t controlWord);
	uint16_t _statusWord;
	int8_t _modesOfOperation;				uint32_t SetModesOfOperation(int8_t modesOfOperation);
	int8_t _modesOfOperationDisplay;
	uint16_t _errorCode;
	
	int16_t _targetCurrent;					uint32_t SetTargetCurrent(int32_t targetCurrent);
	int16_t _actualCurrent;
	int32_t _averagedCurrent;
			
	int16_t _targetTorque;					uint32_t SetTargetTorque(int16_t targetTorque);
	int16_t _torqueOffset;					uint32_t SetTorqueOffset(int16_t torqueOffset);
	int16_t _demandTorque;
	int16_t _actualTorque;
	
	int32_t _targetVelocity;				uint32_t SetTargetVelocity(int32_t targetVelocity);
											uint32_t SetTargetProfileVelocity(int32_t targetProfileVelocity, uint32_t maxProfileVelocity, uint32_t acceleration, uint32_t deceleration);
	int32_t _velocityOffset;				uint32_t SetVelocityOffset(int32_t velocityOffset);
	int32_t _demandVelocity;
	int32_t _actualVelocity;
	
	int32_t _targetPosition;				uint32_t SetTargetPosition(int32_t targetPosition);
											uint32_t SetTargetProfilePosition(uint16_t controlWord);
	int32_t _currentTargetPosition;
	int32_t _nextTargetPosition;
	int32_t _demandPosition;
	int32_t _actualPosition;
	int32_t _positionFollowingError;
	
	uint32_t _dcLinkVoltage;
	uint8_t _hallSensorPattern;
	int32_t _loadTorque;
	int32_t _loadTorqueThreshold;		uint32_t SetLoadTorqueThreshold(int32_t loadTorqueThreshold);
	uint8_t _temperature;																			

	//uint8_t _fHallSameCntClear;
	//uint32_t _hallSameCnt; 			int8_t CheckHallSensorErr(void); //pjg++180827
	//uint8_t _fEncSameCntClear;
	//uint32_t _encSameCnt;			int8_t CheckEncoderErr(void); //pjg++180827
	float _actualVelocityCoe; //pjg++190508
	int32_t _oldActualVelocity;  //pjg++190508
	uint16_t _actualVelocityFrqOfLPF; 	uint32_t SetActualVelocityFrqOfLPF(uint16_t freq);   //pjg++190508
	
	
	
	void UpdateStatusWord(void);
	uint32_t SetEnable(void);
	void SetErrorCode(uint16_t errorCode);
	uint32_t SetHalt(void);
	uint32_t SetQuickStop(void);
	uint32_t SetHoming(void);
	void RunHoming(void);
	
	
	
	//	Parameters
	//uint8_t _id;	//pjg--180417 duplicate to	CanOpenCia301						

	uint32_t SetCANID(uint8_t id); //pjg++180417
		
	//	Motor Properties
	uint16_t _motorType;
	uint16_t _maximumCurrent;					uint32_t SetMaximumCurrent(uint16_t maximumCurrent);
	uint32_t _ratedCurrent;						uint32_t SetRatedCurrent(uint32_t ratedCurrent);
	
	int16_t _maximumTorque;						uint32_t SetMaximumTorque(int16_t maximumTorque);
	uint32_t _ratedTorque;						uint32_t SetRatedTorque(uint32_t ratedTorque);
	
	uint32_t _maxMotorSpeed;					uint32_t SetMaxMotorSpeed(uint32_t maxMotorSpeed);
	
	uint32_t _positionEncoderIncrement;		
	uint32_t _positionEncoderMotorRevolution;
	
	int16_t _motionProfileType;					uint32_t SetMotionProfileType(int16_t motionProfileType);
	uint32_t _maxProfileVelocity;				uint32_t SetMaxProfileVelocity(uint32_t maxProfileVelocity);
	uint32_t _profileVelocity;					uint32_t SetProfileVelocity(uint32_t profileVelocity);
	uint32_t _profileAcceleration;				uint32_t SetProfileAcceleration(uint32_t profileAcceleration);
	uint32_t _profileDeceleration;				uint32_t SetProfileDeceleration(uint32_t profileDeceleration);
	uint32_t _quickStopDeceleration;			uint32_t SetQuickStopDeceleration(uint32_t quickStopDeceleration);
	uint32_t _maxAcceleration;					uint32_t SetMaxAcceleration(uint32_t maxAcceleration);
	uint32_t _maxDeceleration;					uint32_t SetMaxDeceleration(uint32_t maxDeceleration);
	
	int32_t _minSwPositionLimit;				uint32_t SetMinSwPositionLimit(int32_t minSwPositionLimit);
	int32_t _maxSwPositionLimit;				uint32_t SetMaxSwPositionLimit(int32_t maxSwPositionLimit);
	
	uint16_t _windowTime;
	uint32_t _positionWindow;					uint32_t SetPositionWindow(uint32_t positionWindow);
	uint16_t _positionWindowTime;				uint32_t SetPositionWindowTime(uint16_t positionWindowTime);
	uint32_t _velocityWindow;					uint32_t SetVelocityWindow(uint32_t velocityWindow);
	uint16_t _velocityWindowTime;				uint32_t SetVelocityWindowTime(uint16_t velocityWindowTime);
	
	uint32_t _followingErrorWindow;				uint32_t SetFollowingErrorWindow(uint32_t followingErrorWindow);
	
	int8_t _positionNotationIndex;				uint32_t SetPositionNotationIndex(int8_t positionNotationIndex);
	uint8_t _positionDimensionIndex;			uint32_t SetPositionDimensionIndex(uint8_t positionDimensionIndex);
	int8_t _velocityNotationIndex;				uint32_t SetVelocityNotationIndex(int8_t velocityNotationIndex);
	uint8_t _velocityDimensionIndex;			uint32_t SetVelocityDimensionIndex(uint8_t velocityDimensionIndex);
	int8_t _accelerationNotationIndex;			uint32_t SetAccelerationNotationIndex(int8_t accelerationNotationIndex);
	uint8_t _accelerationDimensionIndex;		uint32_t SetAccelerationDimensionIndex(uint8_t accelerationDimensionIndex);
	
	int8_t _homingMethod;						uint32_t SetHomingMethod(int8_t homingMethod);
	uint32_t _switchSearchVelocity;				uint32_t SetSwitchSearchVelocity(uint32_t switchSearchVelocity);
	uint32_t _zeroSearchVelocity;				uint32_t SetZeroSearchVelocity(uint32_t zeroSearchVelocity);
	uint32_t _homingAcceleration;				uint32_t SetHomingAcceleration(uint32_t homingAcceleration);
	int32_t _homeOffset;						uint32_t SetHomeOffset(int32_t homeOffset);
	uint8_t _homingStatus;
	uint32_t _prevHomeSwitchStatus;
	
	uint16_t _currentPGain;						uint32_t SetCurrentPGain(uint16_t currentPGain);					//	Number		10^-3
	uint16_t _currentIGain;						uint32_t SetCurrentIGain(uint16_t currentIGain);					//	Number		10^0
	int16_t _currentOffset;						uint32_t SetCurrentOffset(int16_t currentOffset);				//	Number		0.01A  //pjg++181130
	uint16_t _velocityPGain;					uint32_t SetVelocityPGain(uint16_t velocityPGain);					//	Number		10^-4
	uint16_t _velocityIGain;					uint32_t SetVelocityIGain(uint16_t velocityIGain);					//	Number		10^-3
	uint16_t _positionPGain;					uint32_t SetPositionPGain(uint16_t positionPGain);					//	Number		10^-2
		
	uint32_t _digitalInput;
	uint32_t _digitalOutput;					uint32_t SetDigitalOutput(uint32_t digitalOutput);
	uint32_t _digitalOutputMask;				uint32_t SetDigitalOutputMask(uint32_t digitalOutputMask);
	uint16_t _analogInput[4];					uint16_t GetAnalogIntput(uint8_t index);
	
	//	Kitech Device Profile
	uint16_t _resistance;						uint32_t SetResistance(uint16_t resistance);					//	m¥Ø, 		10^-3
	uint32_t _qAxisInductance;					uint32_t SetQAxisInductance(uint32_t qAxisInductance);			//	uH, 		10^-6
	uint32_t _dAxisInductance;					uint32_t SetDAxisInductance(uint32_t dAxisInductance);			//	uH, 		10^-6
	uint32_t _torqueConstant;					uint32_t SetTorqueConstant(uint32_t torqueConstant);			//	uNm/A,		10^-6
	uint32_t _backEmfConstant;					uint32_t SetBackEmfConstant(uint32_t backEmfConstant);			//	uV/(rad/s)	10^-6
	uint32_t _systemInertia;					uint32_t SetSystemInertia(uint32_t systemInertia);				//	mgcm^2		10^-10
	uint32_t _coulombFriction;					uint32_t SetCoulombFriction(uint32_t coulombFriction);			//	uNm			10^-6
	uint32_t _viscosFriction;					uint32_t SetViscosFriction(uint32_t viscosFriction);			//	uNm/(rad/s)	10^-6
	int16_t _electricAngleOffset;				uint32_t SetElectricAngleOffset(int16_t electricAngleOffset);	//	degree
	uint8_t _motorPhase;						uint32_t SetMotorPhase(uint8_t motorPhase);						//	Number		0 ~ 5
	
	uint8_t _positionSensorType;
	uint8_t _positionSensorPolarity;
	uint8_t _polePairs;							uint32_t SetPolePair(uint8_t polePair);
	
	int32_t _dAxisVoltage;						uint32_t SetDAxisVoltage(int32_t dAxisVoltage);
	int32_t _qAxisVoltage;						uint32_t SetQAxisVoltage(int32_t qAxisVoltage);
	
	int32_t _dAxisTargetCurrent;
	int32_t _qAxisTargetCurrent;
	int32_t _dAxisActualCurrent;
	int32_t _qAxisActualCurrent;
	
	uint32_t _velocityFollowingError;			uint32_t SetVelocityFollowingError(uint32_t velocityFollowingError);
	
	uint8_t _velocityAutoTuningAcceleration;	uint32_t SetVelocityAutoTuningAcceleration(uint8_t velocityAutoTuningAcceleration);
	uint8_t _velocityAutoTuningVelocity;		uint32_t SetVelocityAutoTuningVelocity(uint8_t velocityAutoTuningVelocity);
	int32_t _velocityAutoTuningPosition;		uint32_t SetVelocityAutoTuningPosition(int32_t velocityAutoTuningPosition);
	uint16_t _velocityControllerBandwidth;		uint32_t SetVelocityControllerBandwidth(uint16_t velocityControllerBandwidth);
	
	uint8_t _pcpModeStatus;
	uint16_t _pcpModeCurrentModeRunTime;
	int32_t _pcpModeHomePosition;				uint32_t SetPcpModeHomePosition(int32_t pcpModeHomePosition);
	int32_t _pcpModeTargetPosition;				uint32_t SetPcpModeTargetPosition(int32_t pcpModeTargetPosition);
	int32_t _pcpModeTargetCurrent;				uint32_t SetPcpModeTargetCurrent(int32_t pcpModeTargetCurrent);
	uint16_t _pcpModeCurrentModeDuration;		uint32_t SetPcpModeCurrentModeDuration(uint16_t pcpModeCurrentModeDuration);
	
	
	uint16_t _brakeDurationTime;
	uint16_t _brakeRunTime;
	
	int16_t _bufIndex;
	int32_t _buf0[BUF_SIZE];					int32_t GetBuf0(uint16_t index);
	int32_t _buf1[BUF_SIZE];					int32_t GetBuf1(uint16_t index);

	//etc
	uint32_t _motorInfoSendType, _curMotorInfoSendType;
	uint32_t _bkId;
	uint32_t _digitalInputMask; //pjg++190503
	uint32_t _digitalInputPolarity; //pjg++190503

	uint32_t SetMotorInfoSendType(uint32_t type);
	//uint16_t GetMotorInfoSendType(void);
	//uint16_t GetCurMotorInfoSendType(void);
	//void ResetMotorInfoSendType(void);
	int MakeMotorInfoSendData(uint8_t *data);
	uint32_t SetDigitalInputMask(uint32_t mask); //pjg++190506
	uint32_t SetDigitalInputPolarity(uint32_t polarity); //pjg++190506
	
	//	Callback Functions
	OnCia402StateMachineChangnedFunc _Fault_To_SwitchOnDisabled;
	OnCia402StateMachineChangnedFunc _SwitchOnDisalbed_TO_ReadyToSwitchOn;
	OnCia402StateMachineChangnedFunc _ReadyToSwitchOn_TO_SwitchOnDisabled;
	OnCia402StateMachineChangnedFunc _ReadyToSwitchOn_TO_SwitchedOn;
	OnCia402StateMachineChangnedFunc _SwitchedOn_To_SwitchOnDisabled;
	OnCia402StateMachineChangnedFunc _SwitchedOn_To_ReadyToSwitchOn;
	OnCia402StateMachineChangnedFunc _SwitchedOn_To_OperationEnable;
	OnCia402StateMachineChangnedFunc _OperationEnable_To_SwitchOnDisabled;
	OnCia402StateMachineChangnedFunc _OperationEnable_To_ReadyToSwitchOn;
	OnCia402StateMachineChangnedFunc _OperationEnable_To_SwitchedOn;
	OnCia402StateMachineChangnedFunc _OperationEnable_To_QuickStopActive;
	OnCia402StateMachineChangnedFunc _QuickStopActive_To_SwitchOnDisabled;
	OnCia402StateMachineChangnedFunc _QuickStopActive_To_OperationEnable;
	
	SetTargetVoltageFunc _setTargetVoltage;
	SetTargetCurrentFunc _setTargetCurrent;
	SetTargetTorqueFunc _setTargetTorque;
	SetTargetVelocityFunc _setTargetVelocity;
	SetTargetProfileVelocityFunc _setTargetProfileVelocity;
	SetTargetPositionFunc _setTargetPosition;
	SetTargetProfilePositionFunc _setTargetProfilePosition;
	SetFaultFunc _setFault;
	SetPositionEncoderSensorValueFunc _setPositionEncoderSensorValue;
	
	RunBrakeFunc _runBrake;
	
protected:
	void HomingMethodPlus17(void);
	void HomingMethodPlus18(void);
	void HomingMethodPlus23(void);
	void HomingMethodPlus27(void);
	void HomingMethodPlus37(void);
	void HomingMethodMinus03(void);
	void HomingMethodMinus04(void);
};
