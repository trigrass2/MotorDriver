#include "Peripheral.h"
#include "ServoControllerInterface.h"
#include "ServoController.h"
#include "CanOpen.h"
#include "Io.h"
#include "utils.h"

#define PARAM_VER				0x0100 //v01.00	//pjg++190426

CanOpen canOpen(0x00000892, 0x000000FF, 0x20161010, 0x20161010);
ServoController servoController(&canOpen);

//	SDO
int8_t SetInt8Data(uint16_t index, uint8_t subIndex, int8_t data);
int8_t SetUint8Data(uint16_t index, uint8_t subIndex, uint8_t data);
int8_t SetInt16Data(uint16_t index, uint8_t subIndex, int16_t data);
int8_t SetUint16Data(uint16_t index, uint8_t subIndex, uint16_t data);
int8_t SetInt32Data(uint16_t index, uint8_t subIndex, int32_t data);
int8_t SetUint32Data(uint16_t index, uint8_t subIndex, uint32_t data);

int8_t GetInt8Data(uint16_t index, uint8_t subIndex);
uint8_t GetUint8Data(uint16_t index, uint8_t subIndex);
int16_t GetInt16Data(uint16_t index, uint8_t subIndex);
uint16_t GetUint16Data(uint16_t index, uint8_t subIndex);
int32_t GetInt32Data(uint16_t index, uint8_t subIndex);
uint32_t GetUint32Data(uint16_t index, uint8_t subIndex);

//	CiA-402 State Machine
int8_t Cia402_Fault_To_SwitchOnDisabled(void);
int8_t Cia402_SwitchOnDisalbed_TO_ReadyToSwitchOn(void);
int8_t Cia402_ReadyToSwitchOn_TO_SwitchOnDisabled(void);
int8_t Cia402_ReadyToSwitchOn_TO_SwitchedOn(void);
int8_t Cia402_SwitchedOn_To_SwitchOnDisabled(void);
int8_t Cia402_SwitchedOn_To_ReadyToSwitchOn(void);
int8_t Cia402_SwitchedOn_To_OperationEnable(void);
int8_t Cia402_OperationEnable_To_SwitchOnDisabled(void);
int8_t Cia402_OperationEnable_To_ReadyToSwitchOn(void);
int8_t Cia402_OperationEnable_To_SwitchedOn(void);
int8_t Cia402_OperationEnable_To_QuickStopActive(void);
int8_t Cia402_QuickStopActive_To_SwitchOnDisabled(void);
int8_t Cia402_QuickStopActive_To_OperationEnable(void);

//	Control Command
int8_t SetTargetVoltage(int32_t dAxisVoltage, int32_t qAxisVoltage);
int8_t SetTargetCurrent(int32_t targetCurrent);
int8_t SetTargetTorque(int16_t targetTorque);
int8_t SetTargetVelocity(int32_t targetVelocity);
int8_t SetTargetProfileVelocity(int32_t targetProfileVelocity, uint32_t maxProfileVelocity, uint32_t acceleration, uint32_t deceleration);
int8_t SetTargetPosition(int32_t targetPosition);
int8_t SetTargetProfilePosition(int32_t targetProfilePosition, uint32_t profileVelocity, uint32_t acceleration, uint32_t deceleration);
int8_t SetFault(void);
int8_t SetPositionEncoderSensorValue(int32_t positionEncoderSensorValue);

void RunBrake(uint16_t runTime);

void InitServoController(void)
{
	canOpen._maxContCurrent = (uint16_t)(MAX_CONT_CURRENT*1000.f);
	canOpen._maxOutputCurrent = (uint16_t)(MAX_OUTPUT_CURRENT*1000.f);
	
	canOpen._Fault_To_SwitchOnDisabled = Cia402_Fault_To_SwitchOnDisabled;
	canOpen._SwitchOnDisalbed_TO_ReadyToSwitchOn = Cia402_SwitchOnDisalbed_TO_ReadyToSwitchOn;
	canOpen._ReadyToSwitchOn_TO_SwitchOnDisabled = Cia402_ReadyToSwitchOn_TO_SwitchOnDisabled;
	canOpen._ReadyToSwitchOn_TO_SwitchedOn = Cia402_ReadyToSwitchOn_TO_SwitchedOn;
	canOpen._SwitchedOn_To_SwitchOnDisabled = Cia402_SwitchedOn_To_SwitchOnDisabled;
	canOpen._SwitchedOn_To_ReadyToSwitchOn = Cia402_SwitchedOn_To_ReadyToSwitchOn;
	canOpen._SwitchedOn_To_OperationEnable = Cia402_SwitchedOn_To_OperationEnable;
	canOpen._OperationEnable_To_SwitchOnDisabled = Cia402_OperationEnable_To_SwitchOnDisabled;
	canOpen._OperationEnable_To_ReadyToSwitchOn = Cia402_OperationEnable_To_ReadyToSwitchOn;
	canOpen._OperationEnable_To_SwitchedOn = Cia402_OperationEnable_To_SwitchedOn;
	canOpen._OperationEnable_To_QuickStopActive = Cia402_OperationEnable_To_QuickStopActive;
	canOpen._QuickStopActive_To_SwitchOnDisabled = Cia402_QuickStopActive_To_SwitchOnDisabled;
	canOpen._QuickStopActive_To_OperationEnable = Cia402_QuickStopActive_To_OperationEnable;
	
	canOpen._setInt8DataFunc = SetInt8Data;
	canOpen._setUint8DataFunc = SetUint8Data;
	
	canOpen._setInt16DataFunc = SetInt16Data;
	canOpen._setUint16DataFunc = SetUint16Data;
	
	canOpen._setInt32DataFunc = SetInt32Data;
	canOpen._setUint32DataFunc = SetUint32Data;
	
	canOpen._setTargetVoltage = SetTargetVoltage;
	canOpen._setTargetCurrent = SetTargetCurrent;
	canOpen._setTargetTorque = SetTargetTorque;
	canOpen._setTargetVelocity = SetTargetVelocity;
	canOpen._setTargetProfileVelocity = SetTargetProfileVelocity;
	canOpen._setTargetPosition = SetTargetPosition;
	canOpen._setTargetProfilePosition = SetTargetProfilePosition;
	canOpen._setFault = SetFault;
	canOpen._setPositionEncoderSensorValue = SetPositionEncoderSensorValue;
	
	canOpen._runBrake = RunBrake; 
}

void LoadMotorProperty(uint32_t motorId)
{
	MOTOR_PROPERTY_EX mpex; //pjg++190426
	int index = 0;
    int i;

	for(index = 0; index < NUMBER_OF_MOTOR_PROPERTY; index++) {
		if(gMotorProperties[index].motorId == motorId) {
			break;
		}
	}
	
	if(index == NUMBER_OF_MOTOR_PROPERTY) {
		index = 0;
		//LoadProperties((uint8_t *)&gMotorProperties[index], sizeof(MOTOR_PROPERTY));
		LoadProperties((uint8_t *)&mpex, sizeof(MOTOR_PROPERTY_EX));
		if (mpex.motorProperty.id > 127 || 
			mpex.motorProperty.polePairs > 127 ||
			mpex.motorProperty.motorPhase > 127 ||
			mpex.header[0] != 'H' ||
			mpex.header[1] != 'X' ) {
			for(index = 0; index < sizeof(MOTOR_PROPERTY); index++) {
				*((uint8_t *)&mpex.motorProperty + index) = 0;
			}
			mpex.digitalInputMask = 0xffffffff;
			mpex.digitalInputPolarity = 0x0;
		}
	}
	else {
		//SaveProperties((uint8_t *)&gMotorProperties[index], sizeof(MOTOR_PROPERTY));
		mpex.header[0] = 'H';
		mpex.header[1] = 'X';
		mpex.ver = PARAM_VER;
		mpex.digitalInputMask = 0xffffffff;
		mpex.digitalInputPolarity = 0x0;
		for(i = 0; i < sizeof(MOTOR_PROPERTY); i++) {
			*((uint8_t *)&mpex.motorProperty + i) = *((uint8_t *)&gMotorProperties[index] + i) ;
		}
		SaveProperties((uint8_t *)&mpex, sizeof(MOTOR_PROPERTY_EX));
	}
	
	canOpen._id = mpex.motorProperty.id;
	canOpen._motorType =  mpex.motorProperty.motorType;
	canOpen._resistance =  mpex.motorProperty.resistance;
	canOpen._dAxisInductance = mpex.motorProperty.dAxisInductance;
	canOpen._qAxisInductance = mpex.motorProperty.qAxisInductance;
	canOpen._torqueConstant = mpex.motorProperty.torqueConstant;
	canOpen._backEmfConstant = mpex.motorProperty.backEmfConstant;
	canOpen._systemInertia = mpex.motorProperty.systemInertia;
	canOpen._coulombFriction = mpex.motorProperty.coulombFriction;
	canOpen._viscosFriction = mpex.motorProperty.viscosFriction;
	canOpen._electricAngleOffset = mpex.motorProperty.elecAngleOffset;
	canOpen._motorPhase = mpex.motorProperty.motorPhase;
	
	canOpen._ratedCurrent = mpex.motorProperty.ratedCurrent;
	canOpen._maximumCurrent = mpex.motorProperty.maxCurrent;
	canOpen._ratedTorque = mpex.motorProperty.ratedTorque;
	canOpen._maxMotorSpeed = mpex.motorProperty.maxMotorSpeed;
	
	canOpen._positionSensorType = mpex.motorProperty.positionSensorType;
	canOpen._positionSensorPolarity = mpex.motorProperty.positionSensorPolarity;
	canOpen._polePairs = mpex.motorProperty.polePairs;
	canOpen._positionEncoderIncrement = mpex.motorProperty.positionSensorIncrement;
	canOpen._positionEncoderMotorRevolution = 1;
		
	canOpen._currentPGain = mpex.motorProperty.qAxisCurrentPGain;
	canOpen._currentIGain = mpex.motorProperty.qAxisCurrentIGain;
	canOpen._velocityPGain = mpex.motorProperty.velocityPGain;
	canOpen._velocityIGain = mpex.motorProperty.velocityIGain;
	canOpen._positionPGain = mpex.motorProperty.positionPGain;
	
	canOpen._motionProfileType = mpex.motorProperty.motionProfileType;
	canOpen._maxProfileVelocity = mpex.motorProperty.maxProfileVelocity;
	canOpen._profileVelocity = mpex.motorProperty.profileVelocity;
	canOpen._profileAcceleration = mpex.motorProperty.profileAcceleration;
	canOpen._profileDeceleration = mpex.motorProperty.profileDeceleration;
	canOpen._quickStopDeceleration = mpex.motorProperty.quickStopDeceleration;
	canOpen._maxAcceleration = mpex.motorProperty.maxAcceleration;
	canOpen._maxDeceleration = mpex.motorProperty.maxDeceleration;
	
	canOpen._minSwPositionLimit = mpex.motorProperty.minSwPositionLimit;
	canOpen._maxSwPositionLimit = mpex.motorProperty.maxSwPositionLimit;
	
	canOpen._homingMethod = mpex.motorProperty.homingMethod;
	canOpen._switchSearchVelocity = mpex.motorProperty.switchSearchSpeed;
	canOpen._zeroSearchVelocity = mpex.motorProperty.zeroSearchSpeed;
	canOpen._homingAcceleration = mpex.motorProperty.homingAcceleration;
	canOpen._homeOffset = mpex.motorProperty.homeOffset;
	canOpen._digitalInputMask = mpex.digitalInputMask;
	canOpen._digitalInputPolarity = mpex.digitalInputPolarity;
		
	servoController.LoadProperty();
}

int8_t SaveMotorProperty(void)
{
	//MOTOR_PROPERTY motorProperty;
	MOTOR_PROPERTY_EX mpex;

	mpex.header[0] = 'H';
	mpex.header[1] = 'X';
	mpex.ver = PARAM_VER;
	mpex.motorProperty.id = canOpen._id;
	mpex.motorProperty.motorType = canOpen._motorType;
	mpex.motorProperty.resistance = canOpen._resistance;
	mpex.motorProperty.dAxisInductance = canOpen._dAxisInductance;
	mpex.motorProperty.qAxisInductance = canOpen._qAxisInductance;
	mpex.motorProperty.torqueConstant = canOpen._torqueConstant;
	mpex.motorProperty.backEmfConstant = canOpen._backEmfConstant;
	mpex.motorProperty.systemInertia = canOpen._systemInertia;
	mpex.motorProperty.coulombFriction = canOpen._coulombFriction;
	mpex.motorProperty.viscosFriction = canOpen._viscosFriction;
	mpex.motorProperty.elecAngleOffset = canOpen._electricAngleOffset;
	mpex.motorProperty.motorPhase = canOpen._motorPhase;
	
	mpex.motorProperty.ratedCurrent = canOpen._ratedCurrent;
	mpex.motorProperty.maxCurrent = canOpen._maximumCurrent;
	mpex.motorProperty.ratedTorque = canOpen._ratedTorque;
	mpex.motorProperty.maxMotorSpeed = canOpen._maxMotorSpeed;
	
	mpex.motorProperty.positionSensorType = canOpen._positionSensorType;
	mpex.motorProperty.positionSensorPolarity = canOpen._positionSensorPolarity;
	mpex.motorProperty.polePairs = canOpen._polePairs;
	mpex.motorProperty.positionSensorIncrement = canOpen._positionEncoderIncrement;
			
	mpex.motorProperty.qAxisCurrentPGain = canOpen._currentPGain;
	mpex.motorProperty.qAxisCurrentIGain = canOpen._currentIGain;
	mpex.motorProperty.velocityPGain = canOpen._velocityPGain;
	mpex.motorProperty.velocityIGain = canOpen._velocityIGain;
	mpex.motorProperty.positionPGain = canOpen._positionPGain;
	
	mpex.motorProperty.motionProfileType = canOpen._motionProfileType;
	mpex.motorProperty.maxProfileVelocity = canOpen._maxProfileVelocity;
	mpex.motorProperty.profileVelocity = canOpen._profileVelocity;
	mpex.motorProperty.profileAcceleration = canOpen._profileAcceleration;
	mpex.motorProperty.profileDeceleration = canOpen._profileDeceleration;
	mpex.motorProperty.quickStopDeceleration = canOpen._quickStopDeceleration;
	mpex.motorProperty.maxAcceleration = canOpen._maxAcceleration;
	mpex.motorProperty.maxDeceleration = canOpen._maxDeceleration;
	
	mpex.motorProperty.minSwPositionLimit = canOpen._minSwPositionLimit;
	mpex.motorProperty.maxSwPositionLimit = canOpen._maxSwPositionLimit;
	
	mpex.motorProperty.homingMethod = canOpen._homingMethod;
	mpex.motorProperty.switchSearchSpeed = canOpen._switchSearchVelocity;
	mpex.motorProperty.zeroSearchSpeed = canOpen._zeroSearchVelocity;
	mpex.motorProperty.homingAcceleration = canOpen._homingAcceleration;
	mpex.motorProperty.homeOffset = canOpen._homeOffset;
	mpex.digitalInputMask = canOpen._digitalInputMask;
	mpex.digitalInputPolarity = canOpen._digitalInputPolarity;
	
	//if(SaveProperties((uint8_t *)&motorProperty, sizeof(MOTOR_PROPERTY)) < 0) {
	if(SaveProperties((uint8_t *)&mpex, sizeof(MOTOR_PROPERTY_EX)) < 0) {
		return -1;
	}
	
	return 0;
}

uint8_t GetEncoder1Type(void)
{
	return canOpen._positionSensorType;
}

uint8_t GetEncoder2Type(void)
{
	return 0x00;
}

void CalculateElecAngle(uint8_t hallStatus, int32_t encoderPulse)
{
	servoController.CalculateElecAngle(hallStatus, encoderPulse);
}

void SetElecTheta(float elecTheta)
{
	servoController.SetElecTheta(elecTheta);
}

void RunCurrentController(int16_t adc0, int16_t adc1)
{
	servoController.RunCurrentController(adc0, adc1);
}

uint8_t RunVelocityPositionController(void)
{
	servoController.RunVelocityPositionController();
	
	return 0;
}

void CalculateVelocity(int32_t encoderPulse)
{
	servoController.CalculateVelocity(encoderPulse);
}

void CalculateLoadTorque(void)
{
	servoController.CalculateLoadTorque();
}

void CalculateVoltageTemperature(uint16_t voltage, uint16_t temperature)
{
	servoController.CalculateVoltage(voltage);
	servoController.CalculateTemperature(temperature);
}

void SetAnalogInput(uint16_t *analogInput)
{
	servoController.SetAnalogInput(analogInput);
}

int8_t ProcessSdo(CAN_OPEN_SDO_MSG *canOpenSdoMsg)
{
	return canOpen.ProcessSdo(canOpenSdoMsg);
}

int8_t ProcessSdoCan(uint32_t *id, uint8_t *data, uint32_t *len)
{
	return canOpen.ProcessSdoCanOpen(id, data, len);
}

///////////////////////////////////////////////////////////////////////////////
//
//	Callback Function for CiA-402 State Machine
//
///////////////////////////////////////////////////////////////////////////////


//	SwitchOnDisabled
//Drive initialization is complete
//Drive parameters may be changed Drive function is disabled
int8_t Cia402_Fault_To_SwitchOnDisabled(void)
{
	servoController.Disable();
	servoController.ClearFault();
	
	TurnOnBrake();
	
	return 0;
}

int8_t Cia402_ReadyToSwitchOn_TO_SwitchOnDisabled(void)
{
	servoController.Disable();
	
	TurnOnBrake();
	
	return 0;
}

int8_t Cia402_SwitchedOn_To_SwitchOnDisabled(void)
{
	servoController.Disable();
	
	TurnOnBrake();
	
	return 0;
}

int8_t Cia402_OperationEnable_To_SwitchOnDisabled(void)
{
	servoController.Disable();
	
	TurnOnBrake();
	
	return 0;
}

int8_t Cia402_QuickStopActive_To_SwitchOnDisabled(void)
{
	servoController.Disable();
	
	TurnOnBrake();
	
	return 0;
}

//	ReadyToSwitchOn
//Drive parameters may be changed
//Drive function is disabled
int8_t Cia402_SwitchOnDisalbed_TO_ReadyToSwitchOn(void)
{
	TurnOnBrake();
	
	return 0;
}

int8_t Cia402_SwitchedOn_To_ReadyToSwitchOn(void)
{
	TurnOnBrake();
	
	return 0;
}

int8_t Cia402_OperationEnable_To_ReadyToSwitchOn(void)
{
	servoController.Disable();
	
	TurnOnBrake();
	
	return 0;
}

//	SwitchedOn
//Drive function is disabled
int8_t Cia402_ReadyToSwitchOn_TO_SwitchedOn(void)
{
	servoController.Enable();
	
	TurnOnBrake();
	
	return 0;
}

int8_t Cia402_OperationEnable_To_SwitchedOn(void)
{
	servoController.SetCtrlMode(SC_DISABLE_MODE);
	
	TurnOnBrake();
	
	return 0;
}

//	OperationEnable
//No faults have been detected
//Drive function is enabled and power is applied to the motor
int8_t Cia402_SwitchedOn_To_OperationEnable(void)
{
	TurnOffBrake();
	
	return 0;
}

int8_t Cia402_QuickStopActive_To_OperationEnable(void)
{
	return 0;
}

//	QuickStopActive
//Quickstop function is being executed
//Drive function is enabled and power is applied to the motor
int8_t Cia402_OperationEnable_To_QuickStopActive(void)
{
	
	return 0;
}


///////////////////////////////////////////////////////////////////////////////
//
//	Callback Function for Servo Control
//
///////////////////////////////////////////////////////////////////////////////
int8_t SetTargetVoltage(int32_t dAxisVoltage, int32_t qAxisVoltage)
{
	servoController.SetCtrlMode(SC_VOLTAGE_MODE);
	
	servoController.SetDAxisVoltage((float)dAxisVoltage * 0.001f);
	servoController.SetQAxisVoltage((float)qAxisVoltage * 0.001f);
	
	return 0;
}

int8_t SetTargetCurrent(int32_t targetCurrent)
{
	servoController.SetCtrlMode(SC_CURRENT_MODE);
	
	servoController.SetDAxisTargetCurrent(0.0f);
	servoController.SetQAxisTargetCurrent((float)targetCurrent * 0.001f);
		
	return 0;
}

int8_t SetTargetTorque(int16_t targetTorque)
{
	servoController.SetCtrlMode(SC_CURRENT_MODE);

	servoController.SetDAxisTargetCurrent(0.0f);
	servoController.SetQAxisTargetCurrent((float)targetTorque * 0.001f * servoController._ratedCurrent);

	return 0;
}

int8_t SetTargetVelocity(int32_t targetVelocity)
{
	servoController.SetCtrlMode(SC_VELOCITY_MODE);
	if(servoController._motorType == MOTOR_TYPE_LINEAR) {
		servoController.SetTargetVelocity((float)targetVelocity * 0.001f);
	}
	else {
		servoController.SetTargetVelocity((float)targetVelocity * RPM2RPS);
	}
	
	return 0;
}

int8_t SetTargetProfileVelocity(int32_t targetProfileVelocity, uint32_t maxProfileVelocity, uint32_t acceleration, uint32_t deceleration)
{
	servoController.SetCtrlMode(SC_PROFILE_VELOCITY_MODE);
	if(servoController._motorType == MOTOR_TYPE_LINEAR) {
		servoController.SetTargetProfileVelocity((float)targetProfileVelocity * 0.001f, (float)maxProfileVelocity * 0.001f, (float)acceleration * 0.001f, (float)deceleration * 0.001f);
	}
	else {
		servoController.SetTargetProfileVelocity((float)targetProfileVelocity * RPM2RPS, (float)maxProfileVelocity * RPM2RPS, (float)acceleration * RPM2RPS, (float)deceleration * RPM2RPS);
	}
	
	
	return 0;
}

int8_t SetTargetPosition(int32_t targetPosition)
{
	servoController.SetCtrlMode(SC_POSITION_MODE);
	servoController.SetTargetPosition(targetPosition);
	
	return 0;
}

int8_t SetTargetProfilePosition(int32_t targetProfilePosition, uint32_t profileVelocity, uint32_t acceleration, uint32_t deceleration)
{
	servoController.SetCtrlMode(SC_PROFILE_POSITION_MODE);
	
	if(servoController._motorType == MOTOR_TYPE_LINEAR) {
		servoController.SetTargetProfilePosition(targetProfilePosition, (float)profileVelocity * 0.001f, (float)acceleration * 0.001f, (float)deceleration * 0.001f);
	}
	else {
		servoController.SetTargetProfilePosition(targetProfilePosition, (float)profileVelocity * RPM2RPS, (float)acceleration * RPM2RPS, (float)deceleration * RPM2RPS);
	}
	
	return 0;
}

int8_t SetFault(void)
{
	servoController.SetFault();
	TurnOnBrake();
	return 0;
}

int8_t SetPositionEncoderSensorValue(int32_t positionEncoderSensorValue)
{
	servoController.SetPositionEncoderSensorValue(positionEncoderSensorValue);
	return 0;
}

void RunBrake(uint16_t runTime)
{
	ReadyBrake();
}

///////////////////////////////////////////////////////////////////////////////
//
//	Callback Function for Motor Properties
//
///////////////////////////////////////////////////////////////////////////////
int8_t SetInt8Data(uint16_t index, uint8_t subIndex, int8_t data)
{
	if((index == CIA_402_MODES_OF_OPERATION_DISPLAY) && (subIndex == 0x00)) {
		if(data == CIA_402_VELOCITY_AUTO_TUNE_MODE) {
			return servoController.SetCtrlMode(SC_VELOCITY_AUTO_TUNING_MODE);
		}
		else if(data == CIA_402_CURRENT_AUTO_TUNE_MODE) {
			return servoController.SetCtrlMode(SC_CURRENT_AUTO_TUNING_MODE);
		}
	}
	else if((index == KITECH_CIA_402_MOTOR_PHASE) && (subIndex == 0x00)) {
		return servoController.SetMotorPhase(data);
	}
	
	return -1;
}

int8_t SetUint8Data(uint16_t index, uint8_t subIndex, uint8_t data)
{
	if((index == KITECH_CIA_402_HALL_SENSOR_POLE_PAIR) && (subIndex == 0x00)) {
		servoController.SetPolePair(data);
	}
	else {
		return -1;
	}
	
	return 0;
}

int8_t SetInt16Data(uint16_t index, uint8_t subIndex, int16_t data)
{
	if((index == CIA_402_MOTION_PROFILE_TYPE) && (subIndex == 0x00)) {
		servoController.SetMotionProfileType(data);
	}
	else if((index == KITECH_CIA_402_ELECTRIC_ANGLE_OFFSET) && (subIndex == 0x00)) {
		servoController.SetElectricAngleOffset((float)data*0.1f);		//	10^-1
	}
	else if((index == CIA_402_CURRENT_PARAMETER) && (subIndex == CIA_402_CURRENT_OFFSET)) { //pjg++181130
		servoController.SetCurrentOffset((float)data/(float)1000);				//	10^-2
	}
	else {
		return -1;
	}
	
	return 0;
}

int8_t SetUint16Data(uint16_t index, uint8_t subIndex, uint16_t data)
{
	if((index == KITECH_CIA_402_RESISTANCE) && (subIndex == 0x00)) {
		servoController._R = (float)data*0.001f;	//	10^-3
	}
	else if((index == CIA_402_MAXIMUM_CURRENT) && (subIndex == 0x00)) {
		servoController._maximumCurrent = (float)data*0.001f;		//	10^-3
	}
	else if((index == CIA_402_CURRENT_PARAMETER) && (subIndex == CIA_402_CURRENT_P_GAIN)) {
		servoController.SetCurrentPGain((float)data * 0.001f);		//	10^-3
	}
	else if((index == CIA_402_CURRENT_PARAMETER) && (subIndex == CIA_402_CURRENT_I_GAIN)) {
		servoController.SetCurrentIGain((float)data);				//	10^0
	}
	else if((index == CIA_402_VELOCITY_PARAMETER) && (subIndex == CIA_402_VELOCITY_P_GAIN)) {
		if(servoController._motorType == MOTOR_TYPE_LINEAR) {
			servoController.SetVelocityPGain((float)data * 0.01f);	//	10^-2
		}
		else {
			servoController.SetVelocityPGain((float)data * 0.001f);	//	10^-3
		}
	}
	else if((index == CIA_402_VELOCITY_PARAMETER) && (subIndex == CIA_402_VELOCITY_I_GAIN)) {
		if(servoController._motorType == MOTOR_TYPE_LINEAR) { 
			servoController.SetVelocityIGain((float)data * 0.1f);		//	10^-1
		}
		else {
			servoController.SetVelocityIGain((float)data * 0.001f);		//	10^-3
		}
	}
	else if((index == CIA_402_POSITION_PARAMETER) && (subIndex == CIA_402_POSITION_P_GAIN)) {
		servoController.SetPositionPGain((float)data * 0.01f);		//	10^-2
	}
	else {
		return -1;
	}
		
	return 0;
}

int8_t SetInt32Data(uint16_t index, uint8_t subIndex, int32_t data)
{
	return 0;
}

int8_t SetUint32Data(uint16_t index, uint8_t subIndex, uint32_t data)
{
	if((index == CIA_402_MOTOR_RATED_CURRENT) && (subIndex == 0)) {
		servoController.SetRatedCurrent((float)data*0.001f);	//	10^-3
	}
	else if((index == CIA_402_MOTOR_RATED_TORQUE) && (subIndex == 0)) {
		servoController.SetRatedTorque((float)data*0.001f);	//	10^-3
	}
	else if((index == CIA_402_MAXIMUM_MOTOR_SPEED) && (subIndex == 0x00)) {
		if(servoController._motorType == MOTOR_TYPE_LINEAR) {
			servoController._maximumSpeed = (float)data * 0.001f;
		}
		else {
			servoController._maximumSpeed = (float)data * RPM2RPS;
		}
	}
	else if((index == CIA_402_PROFILE_VELOCITY) && (subIndex == 0x00)) {
		if(servoController._motorType == MOTOR_TYPE_LINEAR) {
			servoController.SetProfileVelocity((float)data*0.001f);
		}
		else {
			servoController.SetProfileVelocity((float)data*RPM2RPS);
		}
	}
	else if((index == CIA_402_PROFILE_ACCELERATION) && (subIndex == 0x00)) {
		if(servoController._motorType == MOTOR_TYPE_LINEAR) {
			servoController.SetProfileAcceleration((float)data*0.001f);
		}
		else {
			servoController.SetProfileAcceleration((float)data*RPM2RPS);
		}
	}
	else if((index == CIA_402_PROFILE_DECELERATION) && (subIndex == 0x00)) {
		if(servoController._motorType == MOTOR_TYPE_LINEAR) {
			servoController.SetProfileDeceleration((float)data*0.001f);
		}
		else {
			servoController.SetProfileDeceleration((float)data*RPM2RPS);
		}
	}
	else if((index == CIA_402_QUICK_STOP_DECELERATION) && (subIndex == 0x00)) {
		if(servoController._motorType == MOTOR_TYPE_LINEAR) {
			servoController.SetQuickStopDeceleration((float)data*0.001f);
		}
		else {
			servoController.SetQuickStopDeceleration((float)data*RPM2RPS);
		}
	}
	else if((index == KITECH_CIA_402_Q_AXIS_INDUCTANCE) && (subIndex == 0x00)) {
		servoController._Lq = (float)data*0.000001f;	//	10^-6
	}
	else if((index == KITECH_CIA_402_D_AXIS_INDUCTANCE) && (subIndex == 0x00)) {
		servoController._Ld = (float)data*0.000001f;	//	10^-6
	}
	else if((index == KITECH_CIA_402_TORQUE_CONSTANT) && (subIndex == 0x00)) {
		servoController._Kt = (float)data*0.000001f;	//	10^-6
	}
	else if((index == KITECH_CIA_402_BACK_EMF_CONSTANT) && (subIndex == 0x00)) {
		servoController._Ke = (float)data*0.000001f;	//	10^-6
	}
	else if((index == KITECH_CIA_402_SYSTEM_INERTIA) && (subIndex == 0x00)) {
		if(servoController._motorType == MOTOR_TYPE_LINEAR) {
			servoController._Jm = (float)data*0.000001f;	//	10^-6
		}
		else {
			servoController._Jm = (float)data*0.0000000001f;	//	10^-10
		}
	}
	else if((index == KITECH_CIA_402_COULOMB_FRICTION) && (subIndex == 0x00)) {
		servoController._Fc = (float)data*0.000001f;	//	10^-6
	}
	else if((index == KITECH_CIA_402_VISCOS_FRICTION) && (subIndex == 0x00)) {
		servoController._Fv = (float)data*0.000001f;	//	10^-6
	}
	else if((index == CIA_402_DIGITAL_OUTPUT) && (subIndex == CIA_402_DIGITAL_OUTPUT_PHYSICAL_OUTPUT)) {
		SetDigitalOutput(data);
	}
	else if((index == CIA_301_STORE) && (subIndex == CIA_301_STORE_ALL_PARAMETERS)) {
		return SaveMotorProperty();
	}
	else {
		return -1;
	}
	
	return 0;
}

//pjg++180710
int Callback_MotorInfoSend(uint32_t *id, uint8_t *data, uint32_t *len)
{
	if (!canOpen.MakeMotorInfoSendData(data)) return 0;
	*id = canOpen._bkId;
	//data[0] = SDO_UPLOAD_REQUEST;
	return canOpen.ProcessSdoCanOpen(id, data, len);
}

