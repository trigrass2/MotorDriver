#include "utils.h"
#include "CanOpenCia402.h"

#define	HALL_ERR_CHECK_CNT			10000 //50us*10000=>50ms
#define	ENC_ERR_CHECK_CNT			10000 //50us*10000=>50ms
#define RUN_LED_STANDBY_NUM			20000 //pjg++190809
#define RUN_LED_ENABLE_DIV_NUM		5 //pjg++190809

extern uint16_t nRunLedCnt; //pjg++190509

CanOpenCia402::CanOpenCia402(uint32_t vendorId, uint32_t productCode, uint32_t revisionNumber, uint32_t serialNumber)
: CanOpenCia301(vendorId, productCode, revisionNumber, serialNumber)
{
	_deviceType = 0x00020192;
	
	_id = 0x00;
	
	_motorType = CIA_402_DC_MOTOR;
	
	
	_maximumCurrent = 2000;		//	2000mA			10^-3
	_ratedCurrent = 1000;		//	1000mA			10^-3
	_ratedTorque = 1000;
	_maxMotorSpeed = 1000;		//	RPM
	
	_positionWindow = 10;		//	5pulse
	_positionWindowTime = 10;	//	10msec
	_velocityWindow = 30;		//	30RPM
	_velocityWindowTime = 10;	//	10msec
	
	_controlWord = 0x0000;
	_statusWord = CIA_402_STATUS_BOOTUP;
	_modesOfOperation = CIA_402_PROFILE_POSITION_MODE;
	_modesOfOperationDisplay = CIA_402_PROFILE_POSITION_MODE;
	
	_windowTime = 0;				//	msec
	
	_dcLinkVoltage = 24000;			//	24000[mV] -> 24[V]
	
	_errorCode = 0x0000;
	
	_nOverLoad = 0;					//	count
	_actualCurrent = 0;				//	mA
	
	_maximumTorque = 1000;			//	1000
	_targetTorque = 0;				//	[Unit] -> 0 ~ 1000
	_actualTorque = 0;				//	[Unit] -> 0 ~ 1000
	
	_targetVelocity = 0;			//	RPM
	_demandVelocity = 0;			//	RPM
	_actualVelocity = 0;			//	RPM
	_velocityFollowingError = 500;	//	RPM
		
	_targetPosition = 0;			//	Count
	_currentTargetPosition = 0;		//	Count
	_nextTargetPosition = 0;		//	Count
	_demandPosition = 0;			//	Count
	_actualPosition = 0;			//	Count
	
	_followingErrorWindow = 100000;	//	Pulse
	
	
	//	Variables for motion control
	_motionProfileType = CIA_402_TRAPEZOIDAL_PROFILE;
	
	_maxProfileVelocity = 0;
	_profileVelocity = 0;
	_profileAcceleration = 0;
	_profileDeceleration = 0;
	_quickStopDeceleration = 0;
	_maxAcceleration = 4294967295;
	_maxDeceleration = 4294967295;
	
	_minSwPositionLimit = 0;
	_maxSwPositionLimit = 0;
	
	//	Variables for homing
	_homingStatus = HOMING_STATUS_INIT;
	_homingMethod = CIA_402_HOMING_METHOD_NEGATIVE_LIMIT_SWITCH;
	_switchSearchVelocity = 100;
	_zeroSearchVelocity = 5;
	_homingAcceleration = 1000;
	_homeOffset = 100;
	_prevHomeSwitchStatus = 0x00000000;
	
	_positionEncoderIncrement = 0;
	_positionEncoderMotorRevolution = 0;
	
	_currentPGain = 0;
	_currentIGain = 0;
	_velocityPGain = 0;
	_velocityIGain = 0;
	_positionPGain = 0;
	
	_hallSensorPattern = 0;
	
	_loadTorque = 0;					//	uNm or mN
	_loadTorqueThreshold = 1000;		//	1mNm or 1N
	
	_positionNotationIndex = CIA_402_NOTATION_INDEX_ZERO;
	_positionDimensionIndex = CIA_402_DIMENSION_INDEX_STEPS;
	_velocityNotationIndex = CIA_402_NOTATION_INDEX_ZERO;
	_velocityDimensionIndex = CIA_402_DIMENSION_INDEX_RPM;
	_accelerationNotationIndex = CIA_402_NOTATION_INDEX_ZERO;
	_accelerationDimensionIndex = CIA_402_DIMENSION_INDEX_RPM;
	
	_Fault_To_SwitchOnDisabled = NULL;
	_SwitchOnDisalbed_TO_ReadyToSwitchOn = NULL;
	_ReadyToSwitchOn_TO_SwitchOnDisabled = NULL;
	_ReadyToSwitchOn_TO_SwitchedOn = NULL;
	_SwitchedOn_To_SwitchOnDisabled = NULL;
	_SwitchedOn_To_ReadyToSwitchOn = NULL;
	_SwitchedOn_To_OperationEnable = NULL;
	_OperationEnable_To_SwitchOnDisabled = NULL;
	_OperationEnable_To_ReadyToSwitchOn = NULL;
	_OperationEnable_To_SwitchedOn = NULL;
	_OperationEnable_To_QuickStopActive = NULL;
	_QuickStopActive_To_SwitchOnDisabled = NULL;
	_QuickStopActive_To_OperationEnable = NULL;
	
	_resistance = 1000;			//	1¥Ø				10^-3
	_qAxisInductance = 1000;	//	1000uH			10^-6
	_dAxisInductance = 1000;	//	1000uH			10^-6
	_torqueConstant = 50000;	//	50000uNm/A		10^-6
	_backEmfConstant = 50000;	//	50000uV/(rad/s)	10^-6
	_systemInertia = 100000;	//	100000mgcm^2	10^-10
	_coulombFriction = 0;		//	0uNm			10^-6
	_viscosFriction = 0;		//	0uNm/(rad/s)	10^-6
	_electricAngleOffset = 0;		//	0degree
	
	_positionSensorType = KITECH_CIA_402_INCREMENTAL_ENCODER;
	_positionSensorPolarity = 0x00;
	_polePairs = 0;
	
	_dAxisVoltage = 0;			//	mV
	_qAxisVoltage = 0;			//	mV
	
	_dAxisTargetCurrent = 0;	//	mA
	_qAxisTargetCurrent = 0;	//	mA
	_dAxisActualCurrent = 0;	//	mA
	_qAxisActualCurrent = 0;	//	mA
	
	_targetCurrent = 0;			//	mA
	_averagedCurrent = 0;		//	mA
	
	_digitalInput = 0x00000000;
	_digitalOutput = 0x00000000;
	_digitalOutputMask = 0x00000000;
	
	_analogInput[0] = 0x0000;
	_analogInput[1] = 0x0000;
	_analogInput[2] = 0x0000;
	_analogInput[3] = 0x0000;
	
	_velocityAutoTuningAcceleration	= 100;		//	100%
	_velocityAutoTuningVelocity = 20;			//	20%
	_velocityAutoTuningPosition = 10000;		//	Count
	_velocityControllerBandwidth = 50;			//	rad/s //pjg<>190807 : when motor that inertia is big, do down bandwidth value
	
	
	_pcpModeStatus = PCP_MODE_STATUS_READY;
	_pcpModeCurrentModeRunTime = 0;
	
	_pcpModeHomePosition = -1000;			//	Pulse
	_pcpModeTargetPosition = -30000;		//	Pulse
	_pcpModeTargetCurrent = -100;			//	mA
	_pcpModeCurrentModeDuration = 1000;		//	0.5ms * 1000 = 0.5s
	
	_brakeDurationTime = 0;				//	0.5ms * 2000 = 1s
	_brakeRunTime = 0;
	
	_bufIndex = -1;

	_motorInfoSendType = 0; //pjg++180717
	_curMotorInfoSendType = 0; //pjg++180717
	_bkId = 0; //pjg++180717
	_actualVelocityCoe = 0.0f; //pjg++190508
	_oldActualVelocity = 0; //pjg++190508
	_actualVelocityFrqOfLPF = 0;
	_torqueOffset = 0; //pjg++190809
	//_fHallSameCntClear= 0; //pjg++180828
	//_fEncSameCntClear= 0; //pjg++180828
	//_hallSameCnt = 0; //pjg++180828
	//_encSameCnt = 0; //pjg++180828
	nRunLedCnt = RUN_LED_STANDBY_NUM; //pjg++190514

	
}

uint32_t CanOpenCia402::SetControlWord(uint16_t controlWord)
{
	uint32_t ret = CAN_OPEN_ABORT_CODE_NO_ERROR;
	
    nRunLedCnt = RUN_LED_STANDBY_NUM;
	//	ControlWord Command
	if((controlWord & CIA_402_CONTROL_COMMAND_MASK) != (_controlWord & CIA_402_CONTROL_COMMAND_MASK) ||
		(STATUS_WORD_STATE == CIA_402_STATUS_FAULT)) { //pjg++190508 : not run fault cmd after fault
			//	Not ready to switch on
		if(STATUS_WORD_STATE == CIA_402_STATUS_NOT_READY_TO_SWITCH_ON) {
			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
		}
		//	Fault
		else if(STATUS_WORD_STATE == CIA_402_STATUS_FAULT) {
			//	Fault reset				Fault -> Switch on disabled
			if((controlWord & CIA_402_CONTROL_FAULT_RESET_MASK) == CIA_402_CONTROL_FAULT_RESET) {
				if(_Fault_To_SwitchOnDisabled == NULL)		return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_Fault_To_SwitchOnDisabled() < 0)		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
				
				_errorReg = 0x00;
				_errorCode = 0x0000;
				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_SWITCH_ON_DISABLED;
				_statusWord &= ~CIA_402_HOMING_ERROR; //pjg++190806 : not re-run in error after limit s/w is on
                		_controlWord &= ~CIA_402_CONTROL_FAULT_RESET; //pjg++181001 can do re - fault reset
			}
			//	Error
			else	return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
		}

		//	Switch on disabled
		else if(STATUS_WORD_STATE == CIA_402_STATUS_SWITCH_ON_DISABLED) {
			//	Shutdown				Switch on disabled -> Ready to switch on
			if((controlWord & CIA_402_CONTROL_SHUTDOWN_MASK) == CIA_402_CONTROL_SHUTDONW) {
				if(_SwitchOnDisalbed_TO_ReadyToSwitchOn == NULL)	return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_SwitchOnDisalbed_TO_ReadyToSwitchOn() < 0)		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
				
				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_READY_TO_SWITCH_ON;
			}
			else if((controlWord & CIA_402_CONTROL_DISABLE_VOLTAGE_MASK) == CIA_402_CONTROL_DISABLE_VOLTAGE) { //pjg++190508 : fault -> disable error fix
				if(_ReadyToSwitchOn_TO_SwitchOnDisabled == NULL)	return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_ReadyToSwitchOn_TO_SwitchOnDisabled() < 0)	return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
			
				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_SWITCH_ON_DISABLED;
			}
			//	Error
			else	return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
		}
		
		//	Ready to switch on
		else if(STATUS_WORD_STATE == CIA_402_STATUS_READY_TO_SWITCH_ON) {
			//	Disable voltage			Ready to switch on -> Switch on disabled
			if((controlWord & CIA_402_CONTROL_DISABLE_VOLTAGE_MASK) == CIA_402_CONTROL_DISABLE_VOLTAGE) {
				if(_ReadyToSwitchOn_TO_SwitchOnDisabled == NULL)	return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_ReadyToSwitchOn_TO_SwitchOnDisabled() < 0)		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
			
				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_SWITCH_ON_DISABLED;
			}
			//	Switch on				Ready to switch on -> Switched on
			else if((controlWord & CIA_402_CONTROL_SWITCH_ON_MASK) == CIA_402_CONTROL_SWITCH_ON) {
				if(_ReadyToSwitchOn_TO_SwitchedOn() < 0)	return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
				if(_ReadyToSwitchOn_TO_SwitchedOn() < 0)	return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
				
				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_SWITCH_ON;
			}
			//	Error
			else 	return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
		}
		
		//	Switched on
		else if(STATUS_WORD_STATE == CIA_402_STATUS_SWITCH_ON) {
			//	Disable voltage			Switch on -> Switch on disabled
			if((controlWord & CIA_402_CONTROL_DISABLE_VOLTAGE_MASK) == CIA_402_CONTROL_DISABLE_VOLTAGE) {
				if(_SwitchedOn_To_SwitchOnDisabled == NULL)			return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_SwitchedOn_To_SwitchOnDisabled() < 0)			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
				
				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_SWITCH_ON_DISABLED;
			}
			//	Shutdown				Switch on -> Ready to switch on
			else if((controlWord & CIA_402_CONTROL_SHUTDOWN_MASK) == CIA_402_CONTROL_SHUTDONW) {
				if(_SwitchedOn_To_ReadyToSwitchOn == NULL)			return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_SwitchedOn_To_ReadyToSwitchOn() < 0)			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;

				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_READY_TO_SWITCH_ON;
			}
			//	Enable operation		Switch on -> Operation enabled
			else if((controlWord & CIA_402_CONTROL_ENABLE_OPERATION_MASK) == CIA_402_CONTROL_ENABLE_OPERATION) {
				if(_SwitchedOn_To_OperationEnable == NULL)			return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_SwitchedOn_To_OperationEnable() < 0)			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
				if(_brakeDurationTime != 0) {
					_brakeRunTime = _brakeDurationTime;
				}
				else {
					ret = SetEnable();
					_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_OPERATION_ENABLED;
      				nRunLedCnt = RUN_LED_STANDBY_NUM/RUN_LED_ENABLE_DIV_NUM; //pjg++190509
				}
			}
			//	Error
			else	return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
		}
		//	Operation enabled
		else if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
			//	Disable voltage			Operation enabled -> Switch on disabled
			if((controlWord & CIA_402_CONTROL_DISABLE_VOLTAGE_MASK) == CIA_402_CONTROL_DISABLE_VOLTAGE) {
				if(_OperationEnable_To_SwitchOnDisabled == NULL)	return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_OperationEnable_To_SwitchOnDisabled() < 0)		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
				
				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_SWITCH_ON_DISABLED;
			}
			//	Shutdown				Operation enabled -> Ready to switch on
			else if((controlWord & CIA_402_CONTROL_SHUTDOWN_MASK) == CIA_402_CONTROL_SHUTDONW) {
				if(_OperationEnable_To_ReadyToSwitchOn == NULL)		return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_OperationEnable_To_ReadyToSwitchOn() < 0)		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;

				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_READY_TO_SWITCH_ON;
			}
			//	Disable operation		Operation enabled -> Switch on
			else if((controlWord & CIA_402_CONTROL_DISABLE_OPERATION_MASK) == CIA_402_CONTROL_DISABLE_OPERATION) {
				if(_OperationEnable_To_SwitchedOn == NULL)			return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_OperationEnable_To_SwitchedOn() < 0)			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;

				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_SWITCH_ON;
			}
			//	Quick stop				Operation enabled -> Quick stop active
			else if((controlWord & CIA_402_CONTROL_QUICK_STOP_MASK) == CIA_402_CONTROL_QUICK_STOP) {
				if(_OperationEnable_To_QuickStopActive == NULL)			return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_OperationEnable_To_QuickStopActive() < 0)			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
				
				ret = SetQuickStop();
			}
			//	Control Mode
		}
		//	Quick stop
		else if(STATUS_WORD_STATE == CIA_402_STATUS_QUICK_STOP_ACTIVE) {
			//	Disable voltage			Quick stop active -> Switch on disabled
			if((controlWord & CIA_402_CONTROL_DISABLE_VOLTAGE_MASK) == CIA_402_CONTROL_DISABLE_VOLTAGE) {
				if(_QuickStopActive_To_SwitchOnDisabled == NULL)		return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_QuickStopActive_To_SwitchOnDisabled() < 0)			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
				
				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_SWITCH_ON_DISABLED;
			}
			//	Enable operation		Quick stop active -> Operation enabled
			else if((controlWord & CIA_402_CONTROL_ENABLE_OPERATION_MASK) == CIA_402_CONTROL_ENABLE_OPERATION) {
				if(_QuickStopActive_To_OperationEnable == NULL)			return CAN_OPEN_ABORT_CODE_GENERAL_ERROR;
				if(_QuickStopActive_To_OperationEnable() < 0) 			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;

				_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_OPERATION_ENABLED;
				ret = SetEnable();
      				nRunLedCnt = RUN_LED_STANDBY_NUM/RUN_LED_ENABLE_DIV_NUM; //pjg++190509
			}
			//	Error
			else {
				return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
			}
		}
		//_fHallSameCntClear = 1; //pjg++180830
		//_fEncSameCntClear = 1; //pjg++180830
	}
	
	//	ControlWord Mode
	if((controlWord & CIA_402_CONTROL_ENABLE_OPERATION_MASK) == CIA_402_CONTROL_ENABLE_OPERATION) {
		//	Halt
		if((controlWord & CIA_402_CONTROL_HALT_BIT) == CIA_402_CONTROL_HALT_BIT) {
			ret = SetHalt();
		}
		//	Set New Point
		else if(((controlWord & CIA_402_CONTROL_NEW_SETPOINT) == CIA_402_CONTROL_NEW_SETPOINT) && ((_controlWord & CIA_402_CONTROL_NEW_SETPOINT) != CIA_402_CONTROL_NEW_SETPOINT)) {
			//	Profile Position Mode
			if(_modesOfOperationDisplay == CIA_402_PROFILE_POSITION_MODE) {
				SetTargetProfilePosition(controlWord);
			}
			//	Homing Mode
			else if(_modesOfOperationDisplay == CIA_402_HOMING_MODE) {
				_homingStatus = HOMING_STATUS_START;
				_statusWord &= ~(CIA_402_HOMING_ATTAINED | CIA_402_TARGET_REACHED);
			}
			//	Position-Current-Position Mode
			else if(_modesOfOperationDisplay == CIA_402_POSITION_CURRENT_POSITION_MODE) {
				_pcpModeStatus = PCP_MODE_STATUS_TARGET_POSITION;
				_pcpModeCurrentModeRunTime = 0;
				_statusWord &= ~(CIA_402_TARGET_REACHED | CIA_402_PCP_MODE_ERROR);
				_windowTime = 0;
				ret =  _setTargetProfilePosition(_pcpModeTargetPosition, _profileVelocity, _profileAcceleration, _profileDeceleration);
				_currentTargetPosition = _nextTargetPosition = _pcpModeTargetPosition;
				_demandPosition = _actualPosition;
				_bufIndex = 0;
			}
		}
		//	Auto Tuning
		else if((controlWord & CIA_402_CONTROL_START_AUTO_TUNING) == CIA_402_CONTROL_START_AUTO_TUNING) {
			_statusWord &= ~(CIA_402_SUCCESS_AUTO_TUNING | CIA_402_TARGET_REACHED);
			_setInt8DataFunc(CIA_402_MODES_OF_OPERATION_DISPLAY, 0x00, _modesOfOperationDisplay);
		}
		//_fHallSameCntClear = 0; //pjg++180830
		//_fEncSameCntClear = 0; //pjg++180830
	}
	
	if(ret == CAN_OPEN_ABORT_CODE_NO_ERROR) {
		_controlWord = controlWord;
	}
	
	return ret;
}


uint32_t CanOpenCia402::SetModesOfOperation(int8_t modesOfOperation)
{
	uint32_t ret = CAN_OPEN_ABORT_CODE_NO_ERROR;
	int8_t prevModesOfOperation = _modesOfOperation;
	
	if(modesOfOperation == _modesOfOperationDisplay) {
		return ret;
	}
	
	_modesOfOperation = _modesOfOperationDisplay = modesOfOperation;
	
	
	switch(_modesOfOperationDisplay) {
		case CIA_402_VOLTAGE_MODE :
			if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
				SetDAxisVoltage(_dAxisVoltage);
				ret = SetQAxisVoltage(_qAxisVoltage);
			}
			break;

		case CIA_402_CURRENT_MODE :
			if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
				ret =  SetTargetCurrent(_actualCurrent);
			}
			break;

		case CIA_402_CYCLIC_SYNC_TORQUE_MODE :
		case CIA_402_TORQUE_MODE :
			if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
				ret =  SetTargetTorque(_actualTorque);
			}
			break;
			
		case CIA_402_CYCLIC_SYNC_VELOCITY_MODE :
		case CIA_402_VELOCITY_MODE :
		case CIA_402_PROFILE_VELOCITY_MODE :
			if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
				ret =  SetTargetVelocity(_actualVelocity);
			}
			break;
	
		case CIA_402_CYCLIC_SYNC_POSITION_MODE :
		case CIA_402_POSITION_MODE :
			if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
				ret = SetTargetProfileVelocity(0, _profileVelocity, _profileAcceleration, _profileDeceleration);
				_statusWord |= CIA_402_STOP_AND_POSITION_CONTROL;
			}
			break;
			
		case CIA_402_PROFILE_POSITION_MODE :
			if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
				ret = SetTargetProfileVelocity(0, _profileVelocity, _profileAcceleration, _profileDeceleration);
				_statusWord |= CIA_402_STOP_AND_POSITION_CONTROL;
			}
			break;
	
			
		case CIA_402_HOMING_MODE :
			if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
				_homingStatus = HOMING_STATUS_INIT;
				if(_setTargetProfileVelocity == NULL) {
					ret = CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
				}
				else {
					_setTargetProfileVelocity(0, _maxProfileVelocity, _profileAcceleration, _profileDeceleration);
					_statusWord |= CIA_402_STOP_AND_POSITION_CONTROL;
					_statusWord &= ~CIA_402_CONTROL_HOMING_START;
				}
			}
			else {
				_homingStatus = HOMING_STATUS_READY;
			}
			
			break;
		
		case CIA_402_CURRENT_AUTO_TUNE_MODE :
			_controlWord &= CIA_402_CONTROL_START_AUTO_TUNING;
			_statusWord &= ~(CIA_402_SUCCESS_AUTO_TUNING | CIA_402_TARGET_REACHED);
			break;
			
		case CIA_402_VELOCITY_AUTO_TUNE_MODE :
			if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
				ret =  _setTargetProfilePosition(_actualPosition, _profileVelocity, _profileAcceleration, _profileDeceleration);
			}
			_controlWord &= CIA_402_CONTROL_START_AUTO_TUNING;
			_statusWord &= ~(CIA_402_SUCCESS_AUTO_TUNING | CIA_402_TARGET_REACHED);
			
			break;
			
		case CIA_402_POSITION_AUTO_TUNE_MODE :
			break;
		case CIA_402_POSITION_CURRENT_POSITION_MODE :
			_pcpModeStatus = PCP_MODE_STATUS_HOME_POSITION;
			if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
				_setTargetProfileVelocity(0, _maxProfileVelocity, _profileAcceleration, _profileDeceleration);
				_statusWord |= CIA_402_STOP_AND_POSITION_CONTROL;
			}
			break;
		default :
			return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	if(ret != CAN_OPEN_ABORT_CODE_NO_ERROR) {
		_modesOfOperation = _modesOfOperationDisplay = prevModesOfOperation;
	}
	
	return ret;
}

uint32_t CanOpenCia402::SetTargetTorque(int16_t targetTorque)
{
	if(STATUS_WORD_STATE != CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	if((_modesOfOperationDisplay == CIA_402_CYCLIC_SYNC_TORQUE_MODE) || (_modesOfOperationDisplay == CIA_402_TORQUE_MODE)) {
		if((targetTorque > _maximumTorque) || (targetTorque < -_maximumTorque)) {
			return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
		}
	}
	else {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}

	if(_setTargetTorque == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
	_setTargetTorque(targetTorque);
	
	_targetTorque = targetTorque;
	_bufIndex = 0;
	   
	//_fHallSameCntClear = 0; //pjg++180830
	//_fEncSameCntClear = 0; //pjg++180830
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetTorqueOffset(int16_t torqueOffset)
{
	if((torqueOffset > _maximumTorque) || (torqueOffset < -_maximumTorque)) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_torqueOffset = torqueOffset;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetTargetVelocity(int32_t targetVelocity)
{
	if(STATUS_WORD_STATE != CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	if(targetVelocity == _targetVelocity) return CAN_OPEN_ABORT_CODE_NO_ERROR;
	
	switch(_modesOfOperationDisplay) {
		case CIA_402_CYCLIC_SYNC_VELOCITY_MODE :
		case CIA_402_VELOCITY_MODE :
			if((targetVelocity > (int32_t)_maxMotorSpeed) || (targetVelocity < -(int32_t)_maxMotorSpeed))	return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
			if(_setTargetVelocity == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetVelocity(targetVelocity);
			_targetVelocity = targetVelocity;
			break;
		
		case CIA_402_PROFILE_VELOCITY_MODE :
			return SetTargetProfileVelocity(targetVelocity, _maxProfileVelocity, _profileAcceleration, _profileDeceleration);
			
		default :
			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
			break;
	}
	
	_bufIndex = 0;
	   
	//_fHallSameCntClear = 0; //pjg++180830
	//_fEncSameCntClear = 0; //pjg++180830
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetTargetProfileVelocity(int32_t targetProfileVelocity, uint32_t maxProfileVelocity, uint32_t acceleration, uint32_t deceleration)
{
	if(STATUS_WORD_STATE != CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	if((targetProfileVelocity > (int32_t)_maxProfileVelocity) || (targetProfileVelocity < -(int32_t)_maxProfileVelocity)) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	if(_setTargetProfileVelocity == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
	_setTargetProfileVelocity(targetProfileVelocity, maxProfileVelocity, acceleration, deceleration);
	
	_targetVelocity = targetProfileVelocity;
	
	_bufIndex = 0;
	   
	//_fHallSameCntClear = 0; //pjg++180830
	//_fEncSameCntClear = 0; //pjg++180830
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetTargetPosition(int32_t targetPosition)
{
	//	Check the status
	if(STATUS_WORD_STATE != CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	//	Check the modes of operation
	if(	(_modesOfOperationDisplay == CIA_402_POSITION_MODE) ||
		(_modesOfOperationDisplay == CIA_402_CYCLIC_SYNC_POSITION_MODE) ||
		(_modesOfOperationDisplay == CIA_402_PROFILE_POSITION_MODE)) {
		if(targetPosition > _maxSwPositionLimit) {
			return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
		}
		else if(targetPosition < _minSwPositionLimit) {
			return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
		}
	}
	else {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	//	Set the target position
	if(_modesOfOperationDisplay == CIA_402_PROFILE_POSITION_MODE) {
		_targetPosition = targetPosition;
	}
	else {
		_targetPosition = _currentTargetPosition = _nextTargetPosition = targetPosition;
		
		if(_setTargetPosition == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
		_setTargetPosition(_currentTargetPosition);

		_bufIndex = 0;
	}

	//_fHallSameCntClear = 0; //pjg++180830
	//_fEncSameCntClear = 0; //pjg++180830

	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetTargetProfilePosition(uint16_t controlWord)
{
	_statusWord &= (~CIA_402_STATUS_SET_POINT_ACK);
	
	if((controlWord & CIA_402_CONTROL_NEW_SETPOINT) == CIA_402_CONTROL_NEW_SETPOINT) {
		if((controlWord & CIA_402_CONTROL_RELATIVE_BIT) == CIA_402_CONTROL_RELATIVE_BIT) {
			_nextTargetPosition = _demandPosition + _targetPosition;
		}
		else {
			_nextTargetPosition = _targetPosition;
		}
	}
	
	if((controlWord & CIA_402_CONTROL_CHANGE_SET_IMMEDIATELY) == CIA_402_CONTROL_CHANGE_SET_IMMEDIATELY) {
		_currentTargetPosition = _nextTargetPosition;
		if(_setTargetPosition == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
		_setTargetProfilePosition(_currentTargetPosition, _profileVelocity, _profileAcceleration, _profileDeceleration);
		
		_bufIndex = 0;
	}
	else {
		
	}
	
	_statusWord |= CIA_402_STATUS_SET_POINT_ACK;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetEnable(void)
{
	_dAxisVoltage = _qAxisVoltage = 0;
	_dAxisTargetCurrent = _qAxisTargetCurrent = 0;
	_targetVelocity = _demandVelocity = 0;
	_currentTargetPosition = _nextTargetPosition = _targetPosition = _demandPosition = _actualPosition;
	
	switch(_modesOfOperationDisplay) {
		case CIA_402_VOLTAGE_MODE :
			if(_setTargetVoltage == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetVoltage(0, 0);
			break;

		case CIA_402_CURRENT_MODE :
			if(_setTargetCurrent == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetCurrent(0);
			break;

		case CIA_402_CYCLIC_SYNC_TORQUE_MODE :
		case CIA_402_TORQUE_MODE :
			if(_setTargetTorque == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetTorque(0);
			break;
			
		case CIA_402_CYCLIC_SYNC_VELOCITY_MODE :
		case CIA_402_VELOCITY_MODE :
		case CIA_402_PROFILE_VELOCITY_MODE :
			if(_setTargetProfileVelocity == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetProfileVelocity(0, _maxProfileVelocity, _profileAcceleration, _profileDeceleration);
			break;
	
		case CIA_402_CYCLIC_SYNC_POSITION_MODE :
		case CIA_402_POSITION_MODE :
		case CIA_402_PROFILE_POSITION_MODE :
			if(_setTargetPosition == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_currentTargetPosition = _nextTargetPosition = _demandPosition = _actualPosition;
			_setTargetPosition(_actualPosition);
			break;
	
			
		case CIA_402_HOMING_MODE :
			_homingStatus = HOMING_STATUS_READY;
			if(_setTargetPosition == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_currentTargetPosition = _nextTargetPosition = _demandPosition = _actualPosition;
			_setTargetPosition(_actualPosition);
			break;
		
		case CIA_402_CURRENT_AUTO_TUNE_MODE :
			break;
		case CIA_402_VELOCITY_AUTO_TUNE_MODE :
			if(_setTargetCurrent == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetCurrent(0);
			break;

		case CIA_402_POSITION_AUTO_TUNE_MODE :
			break;
			
		case CIA_402_POSITION_CURRENT_POSITION_MODE :
			if(_setTargetCurrent == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_homingStatus = PCP_MODE_STATUS_READY;
			_setTargetPosition(_actualPosition);
			_currentTargetPosition = _nextTargetPosition = _demandPosition = _actualPosition;
			break;
		default :
			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

void CanOpenCia402::SetErrorCode(uint16_t errorCode)
{
	if(_errorCode != 0x0000) {
		return;
	}
	
	_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_FAULT;
	_errorCode = errorCode;
	
	if(errorCode == CIA_402_ERROR_CODE_OVER_CURRENT_ERROR) {
		_errorReg = CIA_301_ERROR_REG_CURRENT_ERROR;
	}
	else if(errorCode == CIA_402_ERROR_CODE_CURRENT_DETECTION_ERROR) {
		_errorReg = CIA_301_ERROR_REG_GENERIC_ERROR;
	}
	else if(errorCode == CIA_402_ERROR_CODE_FOLLOWING_ERROR) {
		_errorReg = CIA_301_ERROR_REG_DEVICE_PROFILE_ERROR;
	}
	else if(errorCode == CIA_402_ERROR_CODE_OVER_VOLTAGE_ERROR) {
		_errorReg = CIA_301_ERROR_REG_VOLTAGE_ERROR;
	}
	else if(errorCode == CIA_402_ERROR_CODE_UNDER_VOLTAGE_ERROR) {
		_errorReg = CIA_301_ERROR_REG_VOLTAGE_ERROR;
	}
	else if(errorCode == CIA_402_ERROR_CODE_OVER_TEMPERATURE_ERROR) {
		_errorReg = CIA_301_ERROR_REG_TEMPERATURE_ERROR;
	}
	else if(errorCode == CIA_402_ERROR_CODE_GENERIC_ERROR) {
		_errorReg = CIA_301_ERROR_REG_DEVICE_PROFILE_ERROR;
	}
	else if(errorCode == CIA_402_ERROR_CODE_OVER_LOAD_ERROR) {
		_errorReg = CIA_301_ERROR_REG_DEVICE_PROFILE_ERROR;
	}
	
	if(_setFault != NULL) _setFault();
}

uint32_t CanOpenCia402::SetHalt(void)
{
	if(STATUS_WORD_STATE != CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	switch(_modesOfOperationDisplay) {
		case CIA_402_VOLTAGE_MODE :
			if(_setTargetVoltage == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetVoltage(0, 0);
			break;

		case CIA_402_CURRENT_MODE :
			if(_setTargetCurrent == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetCurrent(0);
			break;

		case CIA_402_CYCLIC_SYNC_TORQUE_MODE :
		case CIA_402_TORQUE_MODE :
			if(_setTargetTorque == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetTorque(0);
			break;
			
		case CIA_402_CYCLIC_SYNC_VELOCITY_MODE :
		case CIA_402_VELOCITY_MODE :
		case CIA_402_PROFILE_VELOCITY_MODE :
			if(_setTargetProfileVelocity == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetProfileVelocity(0, _maxProfileVelocity, _profileAcceleration, _profileDeceleration);
			_targetVelocity = 0;
			break;
			
		case CIA_402_HOMING_MODE :
			_homingStatus = HOMING_STATUS_INIT;
	
		case CIA_402_CYCLIC_SYNC_POSITION_MODE :
		case CIA_402_POSITION_MODE :
		case CIA_402_PROFILE_POSITION_MODE :
			if(_setTargetProfileVelocity == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetProfileVelocity(0, _maxProfileVelocity, _profileAcceleration, _profileDeceleration);
			_statusWord |= CIA_402_STOP_AND_POSITION_CONTROL;
			break;
		
		case CIA_402_CURRENT_AUTO_TUNE_MODE :
			break;
		case CIA_402_VELOCITY_AUTO_TUNE_MODE :
			if(_setTargetCurrent == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetCurrent(0);
			break;
		case CIA_402_POSITION_AUTO_TUNE_MODE :
			break;
		default :
			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
			
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetQuickStop(void)
{
	if(STATUS_WORD_STATE != CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	switch(_modesOfOperationDisplay) {
		case CIA_402_VOLTAGE_MODE :
			if(_setTargetVoltage == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetVoltage(0, 0);
			break;

		case CIA_402_CURRENT_MODE :
			if(_setTargetCurrent == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetCurrent(0);
			break;

		case CIA_402_CYCLIC_SYNC_TORQUE_MODE :
		case CIA_402_TORQUE_MODE :
			if(_setTargetTorque == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetTorque(0);
			break;
			
		case CIA_402_CYCLIC_SYNC_VELOCITY_MODE :
		case CIA_402_VELOCITY_MODE :
		case CIA_402_PROFILE_VELOCITY_MODE :
			if(_setTargetProfileVelocity == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetProfileVelocity(0, _maxProfileVelocity, _profileAcceleration, _quickStopDeceleration);
			break;
	
		case CIA_402_HOMING_MODE :
			_homingStatus = HOMING_STATUS_INIT;
			
		case CIA_402_CYCLIC_SYNC_POSITION_MODE :
		case CIA_402_POSITION_MODE :
		case CIA_402_PROFILE_POSITION_MODE :
		
			if(_setTargetProfileVelocity == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetProfileVelocity(0, _maxProfileVelocity, _profileAcceleration, _quickStopDeceleration);
			_statusWord |= CIA_402_STOP_AND_POSITION_CONTROL;
			break;
			
		case CIA_402_CURRENT_AUTO_TUNE_MODE :
			break;
		case CIA_402_VELOCITY_AUTO_TUNE_MODE :
			if(_setTargetCurrent == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
			_setTargetCurrent(0);
			break;
		case CIA_402_POSITION_AUTO_TUNE_MODE :
			break;
		default :
			return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
			
	_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_QUICK_STOP_ACTIVE;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}


uint32_t CanOpenCia402::SetHoming(void)
{
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

void CanOpenCia402::RunHoming(void)
{
	if(_homingStatus == HOMING_STATUS_INIT) {
		if((_statusWord & CIA_402_STOP_AND_POSITION_CONTROL) != CIA_402_STOP_AND_POSITION_CONTROL) {
			_homingStatus = HOMING_STATUS_READY;
			return;
		}
	}
	else if(_homingStatus == HOMING_STATUS_READY) {
		return;
	}
	
	switch(_homingMethod) {
		case CIA_402_HOMING_METHOD_ACTUAL_POSITION :
			//	Homing Method 37
			HomingMethodPlus37();
			break;
		case CIA_402_HOMING_METHOD_HOME_SWITCH_NEGATIVE_SPEED :
			//	Homing Method 27
			HomingMethodPlus27();
			break;
		case CIA_402_HOMING_METHOD_HOME_SWITCH_POSITIVE_SPEED :
			//	HOming Method 23
			HomingMethodPlus23();
			break;
			
		case CIA_402_HOMING_METHOD_POSITIVE_LIMIT_SWITCH :
			//	HOming Method 18
			HomingMethodPlus18();
			break;
			
		case CIA_402_HOMING_METHOD_NEGATIVE_LIMIT_SWITCH :
			//	HOming Method 17
			HomingMethodPlus17();
			break;
			
		case CIA_402_HOMING_METHOD_CURRENT_THRESHOLD_POSITIVE_SPEED :
			//	HOming Method -3
			HomingMethodMinus03();
			break;
			
		case CIA_402_HOMING_METHOD_CURRENT_THRESHOLD_NEGATIVE_SPEED :
			//	HOming Method -4
			HomingMethodMinus04();
			break;
		default :
			break;
	}
	
	if(_homingStatus == HOMING_STATUS_SEARCH_COMPLETE) {
		if(_setPositionEncoderSensorValue != NULL)	_setPositionEncoderSensorValue(0);
		
		_statusWord |= CIA_402_HOMING_ATTAINED;
		
		_currentTargetPosition = _nextTargetPosition = _homeOffset;
		_setTargetProfilePosition(_currentTargetPosition, _zeroSearchVelocity, _homingAcceleration, _homingAcceleration);
		
		_homingStatus = HOMING_STATUS_GO_TO_HOME_POSITION;
	}
	else if(_homingStatus == HOMING_STATUS_GO_TO_HOME_POSITION) {
		int32_t positionError = _currentTargetPosition - _actualPosition;
		
		if((positionError <= (int32_t)_positionWindow) && (positionError >= -(int32_t)_positionWindow)) {
			if(++_windowTime >= _positionWindowTime) {
				if(_setPositionEncoderSensorValue != NULL)	_setPositionEncoderSensorValue(0);
		
				_currentTargetPosition = _nextTargetPosition = _demandPosition = 0;
				_setTargetProfilePosition(_currentTargetPosition, _zeroSearchVelocity, _homingAcceleration, _homingAcceleration);
				_homingStatus = HOMING_STATUS_COMPLETE;
			}
		}
		else {
			_windowTime = 0;
		}
	}
	
	_prevHomeSwitchStatus = _digitalInput;
}

void CanOpenCia402::HomingMethodPlus17(void)
{
	//	Negative Limit Switch
	//	Check Limit Switch
	if((_digitalInput & CIA_402_POSITIVE_LIMIT_BIT) != 0x00000000) {
		_statusWord |= CIA_402_HOMING_ERROR;
		SetErrorCode(CIA_402_ERROR_CODE_FOLLOWING_ERROR);
		return;
	}
		
	if(_homingStatus == HOMING_STATUS_START) {
		if((_digitalInput & CIA_402_NEGATIVE_LIMIT_BIT) != CIA_402_NEGATIVE_LIMIT_BIT) {
			//	Negative Limit Bit : 0
			if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(-_switchSearchVelocity, _switchSearchVelocity, _homingAcceleration, _homingAcceleration);
		}
		else {
			//	Negative Limit Bit : 1
			if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(_switchSearchVelocity, _switchSearchVelocity, _homingAcceleration, _homingAcceleration);
		}
		_homingStatus = HOMING_STATUS_SWITCH_SEARCH;
	}
	else if(_homingStatus == HOMING_STATUS_SWITCH_SEARCH) {
		//	Previous Negative Limit Bit : 0
		if((_prevHomeSwitchStatus & CIA_402_NEGATIVE_LIMIT_BIT) != CIA_402_NEGATIVE_LIMIT_BIT) {
			//	Negative Limit Bit : 0 -> 1
			if((_digitalInput & CIA_402_NEGATIVE_LIMIT_BIT) == CIA_402_NEGATIVE_LIMIT_BIT) {
				if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(_zeroSearchVelocity, _zeroSearchVelocity, _homingAcceleration, _homingAcceleration);
				_homingStatus = HOMING_STATUS_ZERO_SEARCH;
			}
		}
		//	Previous Negative Limit Bit : 1
		else {
			//	Negative Limit Bit : 1 -> 0
			if((_digitalInput & CIA_402_NEGATIVE_LIMIT_BIT) != CIA_402_NEGATIVE_LIMIT_BIT) {
				if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(-_zeroSearchVelocity, _zeroSearchVelocity, _homingAcceleration, _homingAcceleration);
				_homingStatus = HOMING_STATUS_ZERO_SEARCH;
			}
		}
	}
	else if(_homingStatus == HOMING_STATUS_ZERO_SEARCH) {
		//	Previous Negative Limit Bit : 0
		if((_prevHomeSwitchStatus & CIA_402_NEGATIVE_LIMIT_BIT) != CIA_402_NEGATIVE_LIMIT_BIT) {
			//	Negative Limit Bit : 1 -> 0 -> 1
			if((_digitalInput & CIA_402_NEGATIVE_LIMIT_BIT) == CIA_402_NEGATIVE_LIMIT_BIT) {
				_homingStatus = HOMING_STATUS_SEARCH_COMPLETE;
			}
		}
		//	Previous Negative Limit Bit : 1
		else {
			//	Negative Limit Bit : 0 -> 1 -> 0
			if((_digitalInput & CIA_402_NEGATIVE_LIMIT_BIT) != CIA_402_NEGATIVE_LIMIT_BIT) {
				_homingStatus = HOMING_STATUS_SEARCH_COMPLETE;
			}
		}
	}
}

void CanOpenCia402::HomingMethodPlus18(void)
{
	//	Positive Limit Switch
	
	//	Check Limit Switch
	if((_digitalInput & CIA_402_NEGATIVE_LIMIT_BIT) != 0x00000000) {
		_statusWord |= CIA_402_HOMING_ERROR;
		SetErrorCode(CIA_402_ERROR_CODE_FOLLOWING_ERROR);
		return;
	}
		
	if(_homingStatus == HOMING_STATUS_START) {
		if((_digitalInput & CIA_402_POSITIVE_LIMIT_BIT) != CIA_402_POSITIVE_LIMIT_BIT) {
			//	Positive Limit Bit : 0
			if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(_switchSearchVelocity, _switchSearchVelocity, _homingAcceleration, _homingAcceleration);
		}
		else {
			//	Positive Limit Bit : 1
			if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(-_switchSearchVelocity, _switchSearchVelocity, _homingAcceleration, _homingAcceleration);
		}
		_homingStatus = HOMING_STATUS_SWITCH_SEARCH;
	}
	else if(_homingStatus == HOMING_STATUS_SWITCH_SEARCH) {
		//	Previous Positive Limit Bit : 0
		if((_prevHomeSwitchStatus & CIA_402_POSITIVE_LIMIT_BIT) != CIA_402_POSITIVE_LIMIT_BIT) {
			//	Positive Limit Bit : 0 -> 1
			if((_digitalInput & CIA_402_POSITIVE_LIMIT_BIT) == CIA_402_POSITIVE_LIMIT_BIT) {
				if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(-_zeroSearchVelocity, _zeroSearchVelocity, _homingAcceleration, _homingAcceleration);
				_homingStatus = HOMING_STATUS_ZERO_SEARCH;
			}
		}
		//	Previous Positive Limit Bit : 1
		else {
			//	Positive Limit Bit : 1 -> 0
			if((_digitalInput & CIA_402_POSITIVE_LIMIT_BIT) != CIA_402_POSITIVE_LIMIT_BIT) {
				if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(_zeroSearchVelocity, _zeroSearchVelocity, _homingAcceleration, _homingAcceleration);
				_homingStatus = HOMING_STATUS_ZERO_SEARCH;
			}
		}
	}
	else if(_homingStatus == HOMING_STATUS_ZERO_SEARCH) {
		//	Previous Positive Limit Bit : 0
		if((_prevHomeSwitchStatus & CIA_402_POSITIVE_LIMIT_BIT) != CIA_402_POSITIVE_LIMIT_BIT) {
			//	Positive Limit Bit : 1 -> 0 -> 1
			if((_digitalInput & CIA_402_POSITIVE_LIMIT_BIT) == CIA_402_POSITIVE_LIMIT_BIT) {
				_homingStatus = HOMING_STATUS_SEARCH_COMPLETE;
			}
		}
		//	Previous Positive Limit Bit : 1
		else {
			//	Positive Limit Bit : 0 -> 1 -> 0
			if((_digitalInput & CIA_402_POSITIVE_LIMIT_BIT) != CIA_402_POSITIVE_LIMIT_BIT) {
				_homingStatus = HOMING_STATUS_SEARCH_COMPLETE;
			}
		}
	}
}

void CanOpenCia402::HomingMethodPlus23(void)
{
	//	Home Switch Positive Speed
	
	//	Check Limit Switch
	if((_digitalInput & (CIA_402_NEGATIVE_LIMIT_BIT | CIA_402_POSITIVE_LIMIT_BIT)) != 0x00000000) {
		_statusWord |= CIA_402_HOMING_ERROR;
		SetErrorCode(CIA_402_ERROR_CODE_FOLLOWING_ERROR);
		return;
	}
		
	if(_homingStatus == HOMING_STATUS_START) {
		if((_digitalInput & CIA_402_HOME_BIT) != CIA_402_HOME_BIT) {
			//	Home Bit : 0
			if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(_switchSearchVelocity, _switchSearchVelocity, _homingAcceleration, _homingAcceleration);
		}
		else {
			//	Home Bit : 1
			if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(-_switchSearchVelocity, _switchSearchVelocity, _homingAcceleration, _homingAcceleration);
		}
		_homingStatus = HOMING_STATUS_SWITCH_SEARCH;
	}
	else if(_homingStatus == HOMING_STATUS_SWITCH_SEARCH) {
		//	Previous Home Bit : 0
		if((_prevHomeSwitchStatus & CIA_402_HOME_BIT) != CIA_402_HOME_BIT) {
			//	Home Bit : 0 -> 1
			if((_digitalInput & CIA_402_HOME_BIT) == CIA_402_HOME_BIT) {
				if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(-_zeroSearchVelocity, _zeroSearchVelocity, _homingAcceleration, _homingAcceleration);
				_homingStatus = HOMING_STATUS_ZERO_SEARCH;
			}
		}
		//	Previous Home Bit : 1
		else {
			//	Home Bit : 1 -> 0
			if((_digitalInput & CIA_402_HOME_BIT) != CIA_402_HOME_BIT) {
				if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(_zeroSearchVelocity, _zeroSearchVelocity, _homingAcceleration, _homingAcceleration);
				_homingStatus = HOMING_STATUS_ZERO_SEARCH;
			}
		}
	}
	else if(_homingStatus == HOMING_STATUS_ZERO_SEARCH) {
		//	Previous Home Bit : 0
		if((_prevHomeSwitchStatus & CIA_402_HOME_BIT) != CIA_402_HOME_BIT) {
			//	Home Bit : 1 -> 0 -> 1
			if((_digitalInput & CIA_402_HOME_BIT) == CIA_402_HOME_BIT) {
				_homingStatus = HOMING_STATUS_SEARCH_COMPLETE;
			}
		}
		//	Previous Home Bit : 1
		else {
			//	Home Bit : 0 -> 1 -> 0
			if((_digitalInput & CIA_402_HOME_BIT) != CIA_402_HOME_BIT) {
				_homingStatus = HOMING_STATUS_SEARCH_COMPLETE;
			}
		}
	}
}

void CanOpenCia402::HomingMethodPlus27(void)
{
	//	Home Switch Negative Speed

	//	Check Limit Switch
	if((_digitalInput & (CIA_402_NEGATIVE_LIMIT_BIT | CIA_402_POSITIVE_LIMIT_BIT)) != 0x00000000) {
		_statusWord |= CIA_402_HOMING_ERROR;
		SetErrorCode(CIA_402_ERROR_CODE_FOLLOWING_ERROR);
		return;
	}
		
	if(_homingStatus == HOMING_STATUS_START) {
		if((_digitalInput & CIA_402_HOME_BIT) != CIA_402_HOME_BIT) {
			//	Home Bit : 0
			if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(-_switchSearchVelocity, _switchSearchVelocity, _homingAcceleration, _homingAcceleration);
		}
		else {
			//	Home Bit : 1
			if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(_switchSearchVelocity, _switchSearchVelocity, _homingAcceleration, _homingAcceleration);
		}
		_homingStatus = HOMING_STATUS_SWITCH_SEARCH;
	}
	else if(_homingStatus == HOMING_STATUS_SWITCH_SEARCH) {
		//	Previous Home Bit : 0
		if((_prevHomeSwitchStatus & CIA_402_HOME_BIT) != CIA_402_HOME_BIT) {
			//	Home Bit : 0 -> 1
			if((_digitalInput & CIA_402_HOME_BIT) == CIA_402_HOME_BIT) {
				if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(_zeroSearchVelocity, _zeroSearchVelocity, _homingAcceleration, _homingAcceleration);
				_homingStatus = HOMING_STATUS_ZERO_SEARCH;
			}
		}
		//	Previous Home Bit : 1
		else {
			//	Home Bit : 1 -> 0
			if((_digitalInput & CIA_402_HOME_BIT) != CIA_402_HOME_BIT) {
				if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(-_zeroSearchVelocity, _zeroSearchVelocity, _homingAcceleration, _homingAcceleration);
				_homingStatus = HOMING_STATUS_ZERO_SEARCH;
			}
		}
	}
	else if(_homingStatus == HOMING_STATUS_ZERO_SEARCH) {
		//	Previous Home Bit : 0
		if((_prevHomeSwitchStatus & CIA_402_HOME_BIT) != CIA_402_HOME_BIT) {
			//	Home Bit : 1 -> 0 -> 1
			if((_digitalInput & CIA_402_HOME_BIT) == CIA_402_HOME_BIT) {
				_homingStatus = HOMING_STATUS_SEARCH_COMPLETE;
			}
		}
		//	Previous Home Bit : 1
		else {
			//	Home Bit : 0 -> 1 -> 0
			if((_digitalInput & CIA_402_HOME_BIT) != CIA_402_HOME_BIT) {
				_homingStatus = HOMING_STATUS_SEARCH_COMPLETE;
			}
		}
	}
}

void CanOpenCia402::HomingMethodPlus37(void)
{
	if(_homingStatus == HOMING_STATUS_START) {
		_homingStatus = HOMING_STATUS_SEARCH_COMPLETE;
	}
}

void CanOpenCia402::HomingMethodMinus03(void)
{
	if(_homingStatus == HOMING_STATUS_START) {
		_homingStatus = HOMING_STATUS_SWITCH_SEARCH;
		if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(_switchSearchVelocity, _switchSearchVelocity, _homingAcceleration, _homingAcceleration);
	}
	else if(_homingStatus == HOMING_STATUS_SWITCH_SEARCH) {
		if(_nOverLoad >= HOMING_OVER_LOAD_COUNT) {
			_homingStatus = HOMING_STATUS_SEARCH_COMPLETE;
		}
	}
}

void CanOpenCia402::HomingMethodMinus04(void)
{
	if(_homingStatus == HOMING_STATUS_START) {
		_homingStatus = HOMING_STATUS_SWITCH_SEARCH;
		if(_setTargetProfileVelocity != NULL) _setTargetProfileVelocity(-_switchSearchVelocity, _switchSearchVelocity, _homingAcceleration, _homingAcceleration);
	}
	else if(_homingStatus == HOMING_STATUS_SWITCH_SEARCH) {
		if(_nOverLoad >= HOMING_OVER_LOAD_COUNT) {
			_homingStatus = HOMING_STATUS_SEARCH_COMPLETE;
		}
	}
}

void CanOpenCia402::UpdateStatusWord(void)
{
	uint16_t status = _statusWord & CIA_402_STATUS_STATE_MASK;
	int32_t positionError = _currentTargetPosition - _actualPosition;
	int32_t velocityError = _targetVelocity - _actualVelocity;
	
	//	Check the emergency
	if((_digitalInput & BRAKE_SENSOR) != 0x00000000) {
		SetErrorCode(CIA_402_ERROR_CODE_GENERIC_ERROR);
	}
	
	//	Check the Over Load Error
	if(_modesOfOperationDisplay != CIA_402_HOMING_MODE) {	//pjg<>180202 prevent hip overload
		if((_actualCurrent > (int)_maximumCurrent) || (_actualCurrent < -(int)_maximumCurrent)) {
			_nOverLoad++;
		}
		else {
			_nOverLoad--;
		}
	}
	else {
		if((_actualCurrent > (int)_ratedCurrent) || (_actualCurrent < -(int)_ratedCurrent)) {
			_nOverLoad++;
		}
		else {
			_nOverLoad--;
		}
	}
	
	//	Check the Load Torque Threshold
	if((_loadTorque > _loadTorqueThreshold) || (_loadTorque < -_loadTorqueThreshold)) {
		_statusWord |= CIA_402_STATUS_LOAD_TORQUE_THRESHOLD_REACHED;
	}
	else {
		_statusWord &= ~CIA_402_STATUS_LOAD_TORQUE_THRESHOLD_REACHED;
	}

	
	if(_nOverLoad >= MAX_OVER_LOAD_COUNT) {
		//SetErrorCode(CIA_402_ERROR_CODE_OVER_LOAD_ERROR);	//pjg--180206 to use max current (because heat sink is big)
	}
	else if(_nOverLoad < 0) {
		_nOverLoad = 0;
	}
	
	//	Check the limit sensor
	if(_modesOfOperationDisplay != CIA_402_HOMING_MODE) {
		if((_digitalInput & (NEGATIVE_LIMIT_SENSOR || POSITIVE_LIMIT_SENSOR)) != 0x00000000) {
			SetQuickStop();
		}
	}
	
	//	Check Brake
	if(_brakeRunTime != 0) {
		if(--_brakeRunTime == 0) {
			SetEnable();
			_statusWord = (_statusWord & ~CIA_402_STATUS_STATE_MASK) | CIA_402_STATUS_OPERATION_ENABLED;
			_runBrake(0);
		}
	}
			
	//	Quick stop active
	if(STATUS_WORD_STATE == CIA_402_STATUS_QUICK_STOP_ACTIVE) {
		if((_statusWord & CIA_402_STOP_AND_POSITION_CONTROL) == CIA_402_STOP_AND_POSITION_CONTROL) {
			_demandPosition = _actualPosition;
			if((_actualVelocity <= (int32_t)_velocityWindow) && (_actualVelocity >= -(int32_t)_velocityWindow)) {
				if(++_windowTime >= _velocityWindowTime) {
					_windowTime--;
					_statusWord &= (~CIA_402_STOP_AND_POSITION_CONTROL);
					_currentTargetPosition = _nextTargetPosition = _actualPosition;
					if(_setTargetPosition != NULL) _setTargetPosition(_currentTargetPosition);
				}
			}
			else {
				_windowTime = 0;
			}
		}
	}
	//	Operation enabled
	else if(status == CIA_402_STATUS_OPERATION_ENABLED) {
		//	Check the velocity control to position control mode
		if((_statusWord & CIA_402_STOP_AND_POSITION_CONTROL) == CIA_402_STOP_AND_POSITION_CONTROL) {
			_demandPosition = _actualPosition;
			if((_actualVelocity <= (int32_t)_velocityWindow) && (_actualVelocity >= -(int32_t)_velocityWindow)) {
				if(++_windowTime >= _velocityWindowTime) {
					_windowTime--;
					_statusWord &= (~CIA_402_STOP_AND_POSITION_CONTROL);
					_currentTargetPosition = _nextTargetPosition = _demandPosition = _actualPosition;
					if(_setTargetPosition != NULL) _setTargetPosition(_currentTargetPosition);
					//_fHallSameCntClear = 1; //pjg++180828
					//_fEncSameCntClear = 1; //pjg++180828
				}
			}
			else {
				_windowTime = 0;
				//CheckHallSensorErr(); //pjg++180827
				//CheckEncoderErr(); //pjg++180827
			}
		}
		//Check the target reached in profile position mode
		else if(_modesOfOperationDisplay == CIA_402_PROFILE_POSITION_MODE) {
			if((positionError <= (int32_t)_positionWindow) && (positionError >= -(int32_t)_positionWindow)) {
				if(++_windowTime >= _positionWindowTime) {
					_statusWord |= CIA_402_TARGET_REACHED;
					_windowTime--;
					
					if(_currentTargetPosition != _nextTargetPosition) {
						_currentTargetPosition = _nextTargetPosition;
						if(_setTargetProfilePosition != NULL) _setTargetProfilePosition(_currentTargetPosition, _profileVelocity, _profileAcceleration, _profileDeceleration);
					}
				}
				else {
					_statusWord &= ~CIA_402_TARGET_REACHED;
				}
				//_fHallSameCntClear = 1; //pjg++180828
				//_fEncSameCntClear = 1; //pjg++180828
			}
			else {
				_windowTime = 0;
				_statusWord &= ~CIA_402_TARGET_REACHED;
				//CheckHallSensorErr(); //pjg++180827
				//CheckEncoderErr(); //pjg++180827
			}
		}
		//	Check the target reached in profile velocity mode
		else if(_modesOfOperationDisplay == CIA_402_PROFILE_VELOCITY_MODE) {
			if((velocityError <= (int32_t)_velocityWindow) && (velocityError >= -(int32_t)_velocityWindow)) {
				if(++_windowTime >= _velocityWindowTime) {
					_statusWord |= CIA_402_TARGET_REACHED;
					_windowTime--;
				}
				else {
					_statusWord &= ~CIA_402_TARGET_REACHED;
				}
				//_fHallSameCntClear = 1; //pjg++180828
				//_fEncSameCntClear = 1; //pjg++180828
			}
			else {
				_windowTime = 0;
				_statusWord &= ~CIA_402_TARGET_REACHED;
				//CheckHallSensorErr(); //pjg++180827
				//CheckEncoderErr(); //pjg++180827
			}
		}
		//	Run Homing
		else if(_modesOfOperationDisplay == CIA_402_HOMING_MODE) {
			RunHoming();
			
			if(_homingStatus == HOMING_STATUS_COMPLETE)	{
				_statusWord |= CIA_402_TARGET_REACHED;
				//_fHallSameCntClear = 1; //pjg++180828
				//_fEncSameCntClear = 1; //pjg++180828
			}
			else	 {
				_statusWord &= ~CIA_402_TARGET_REACHED;
				//CheckHallSensorErr(); //pjg++180827
				//CheckEncoderErr(); //pjg++180827
			}
		}
		//	Position-Current-Position Mode
		else if(_modesOfOperationDisplay == CIA_402_POSITION_CURRENT_POSITION_MODE) {
			if(_pcpModeStatus == PCP_MODE_STATUS_READY) {
				_windowTime = 0;
			}
			else if(_pcpModeStatus == PCP_MODE_STATUS_TARGET_POSITION) {
				if((_statusWord & CIA_402_STATUS_LOAD_TORQUE_THRESHOLD_REACHED) == CIA_402_STATUS_LOAD_TORQUE_THRESHOLD_REACHED) {
					_setTargetCurrent(_pcpModeTargetCurrent);
					_pcpModeStatus = PCP_MODE_STATUS_TARGET_CURRENT;
					_pcpModeCurrentModeRunTime = 0;
					//_fHallSameCntClear = 1; //pjg++180828
					//_fEncSameCntClear = 1; //pjg++180828
				}
				else if((positionError <= (int32_t)_positionWindow) && (positionError >= -(int32_t)_positionWindow)) {
					if(++_windowTime >= _positionWindowTime) {
						_setTargetProfilePosition(_pcpModeHomePosition, _maxProfileVelocity, _maxAcceleration, _maxDeceleration);
						_currentTargetPosition = _nextTargetPosition = _pcpModeHomePosition;
						_demandPosition = _actualPosition;
						_windowTime = 0;
						_pcpModeStatus = PCP_MODE_STATUS_HOME_POSITION;
						//_fHallSameCntClear = 1; //pjg++180828
						//_fEncSameCntClear = 1; //pjg++180828
					}
				}
				else {
					_windowTime = 0;
					//CheckHallSensorErr(); //pjg++180827
					//CheckEncoderErr(); //pjg++180827
				}
			}
			else if(_pcpModeStatus == PCP_MODE_STATUS_TARGET_CURRENT) {
				if(++_pcpModeCurrentModeRunTime >= _pcpModeCurrentModeDuration) {
					_pcpModeStatus = PCP_MODE_STATUS_HOME_POSITION;
					_setTargetProfilePosition(_pcpModeHomePosition, _maxProfileVelocity, _maxAcceleration, _maxDeceleration);
					_currentTargetPosition = _nextTargetPosition = _pcpModeHomePosition;
					_demandPosition = _actualPosition;
					_windowTime = 0;
					
					_statusWord |= CIA_402_PCP_MODE_ERROR;
					//_fHallSameCntClear = 1; //pjg++180828
					//_fEncSameCntClear = 1; //pjg++180828
				}
				else {
					positionError = _pcpModeTargetPosition - _actualPosition;
					if((positionError <= 10) && (positionError >= -10)) {
						_pcpModeStatus = PCP_MODE_STATUS_HOME_POSITION;
						_setTargetProfilePosition(_pcpModeHomePosition, _maxProfileVelocity, _maxAcceleration, _maxDeceleration);
						_currentTargetPosition = _nextTargetPosition = _pcpModeHomePosition;
						_demandPosition = _actualPosition;
						_windowTime = 0;
						//_fHallSameCntClear = 1; //pjg++180828
						//_fEncSameCntClear = 1; //pjg++180828
					}
					else { //pjg++180827
						//CheckHallSensorErr(); //pjg++180827
						//CheckEncoderErr(); //pjg++180827
					}
				}
			}
			else if(_pcpModeStatus == PCP_MODE_STATUS_HOME_POSITION) {
				if((positionError <= (int32_t)_positionWindow) && (positionError >= -(int32_t)_positionWindow)) {
					if(++_windowTime >= _positionWindowTime) {
						_statusWord |= CIA_402_TARGET_REACHED;
						_pcpModeStatus = PCP_MODE_STATUS_READY;
						_windowTime--;
					}
					else {
						_statusWord &= ~CIA_402_TARGET_REACHED;
					}
					//_fHallSameCntClear = 1; //pjg++180828
					//_fEncSameCntClear = 1; //pjg++180828
				}
				else {
					_windowTime = 0;
					_statusWord &= ~CIA_402_TARGET_REACHED;
					//CheckHallSensorErr(); //pjg++180827
					//CheckEncoderErr(); //pjg++180827
				}
			}
		}
	}
	else {
		_windowTime = 0;
		_statusWord &= ~CIA_402_TARGET_REACHED;

		//_fHallSameCntClear = 1; //pjg++180828
		//_fEncSameCntClear = 1; //pjg++180828
		//_hallSameCnt = 0; //pjg++180827
		//_encSameCnt = 0; //pjg++180827
	}
	
	//	Check the following error
	if((_modesOfOperationDisplay == CIA_402_CYCLIC_SYNC_POSITION_MODE) || (_modesOfOperationDisplay == CIA_402_PROFILE_POSITION_MODE)) {
		_positionFollowingError = _demandPosition - _actualPosition;
		
		if((_positionFollowingError < -(int32_t)_followingErrorWindow) || (_positionFollowingError > (int32_t)_followingErrorWindow)) {
			_statusWord |= CIA_402_FOLLOWING_ERROR;
			SetErrorCode(CIA_402_ERROR_CODE_FOLLOWING_ERROR);
		}
		else {
			_statusWord &= ~CIA_402_FOLLOWING_ERROR;
		}
	}
	else {
		_demandPosition = _actualPosition;
		_positionFollowingError = 0;
	}
}

int32_t CanOpenCia402::GetBuf0(uint16_t index)
{
	if(index >= BUF_SIZE) {
		return 0;
	}
	
	return _buf0[index];
}

int32_t CanOpenCia402::GetBuf1(uint16_t index)
{
	if(index >= BUF_SIZE) {
		return 0;
	}
	
	return _buf1[index];
}
#if 0
//pjg++180827
int8_t CanOpenCia402::CheckHallSensorErr(void)
{
	if (_fHallSameCntClear) _fHallSameCntClear = 0; //pjg++180828
	if (_hallSameCnt > HALL_ERR_CHECK_CNT) {
		SetErrorCode(CIA_402_ERROR_CODE_HALL_SENSOR_ERROR);
	}
	return 1;
}

//pjg++180827
int8_t CanOpenCia402::CheckEncoderErr(void)
{
	if (_fEncSameCntClear) _fEncSameCntClear = 0; //pjg++180828
	if (_encSameCnt > ENC_ERR_CHECK_CNT && _hallSameCnt < HALL_ERR_CHECK_CNT) {
		SetErrorCode(CIA_402_ERROR_CODE_ENCODER_DISCONNECTION_ERROR);
	}
	return 1;
}
#endif

