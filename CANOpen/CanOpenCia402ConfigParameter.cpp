#include "utils.h"
#include "CanOpenCia402.h"

uint32_t CanOpenCia402::SetMaximumCurrent(uint16_t maximumCurrent)
{
	if(maximumCurrent > _maxOutputCurrent) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	else if(maximumCurrent < 2) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	else if(maximumCurrent < _ratedCurrent) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
		
	_setUint16DataFunc(CIA_402_MAXIMUM_CURRENT, 0x00, _maximumCurrent);
	_maximumCurrent = maximumCurrent;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetRatedCurrent(uint32_t ratedCurrent)
{
	if(ratedCurrent > _maxContCurrent) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	else if(ratedCurrent > _maximumCurrent) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR;
	}
	else if(ratedCurrent < 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_setUint32DataFunc(CIA_402_MOTOR_RATED_CURRENT, 0x00, ratedCurrent);
	_ratedCurrent = ratedCurrent;
	_torqueConstant = (uint32_t)((float)_ratedTorque/(float)_ratedCurrent*1000000.0f);
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetMaximumTorque(int16_t maximumTorque)
{
	if((maximumTorque <= 0) || (maximumTorque > 2000)) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}

	_maximumTorque = maximumTorque;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetRatedTorque(uint32_t ratedTorque)
{
	if(ratedTorque < 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_setUint32DataFunc(CIA_402_MOTOR_RATED_TORQUE, 0x00, ratedTorque);
	_ratedTorque = ratedTorque;
	_torqueConstant = (uint32_t)((float)_ratedTorque/(float)_ratedCurrent*1000000.0f);
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetMaxMotorSpeed(uint32_t maxMotorSpeed)
{
	if(maxMotorSpeed > 25000) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR;
	}
	else if(maxMotorSpeed < 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	
	_setUint32DataFunc(CIA_402_MAXIMUM_MOTOR_SPEED, 0x00, maxMotorSpeed);
	_maxMotorSpeed = maxMotorSpeed;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetMotionProfileType(int16_t motionProfileType)
{
	if(!((motionProfileType == 0) || (motionProfileType == 1))) {
		return CAN_OPEN_ABORT_CODE_SERVICE_PARAMETER_ERROR;
	}
	
	_setInt16DataFunc(CIA_402_MOTION_PROFILE_TYPE, 0x00, motionProfileType);
	_motionProfileType = motionProfileType;
	
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetMaxProfileVelocity(uint32_t maxProfileVelocity)
{
	if(maxProfileVelocity > this->_maxMotorSpeed) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR;
	}
	else if(maxProfileVelocity < 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	
	_setUint32DataFunc(CIA_402_MAXIMAL_PROFILE_VELOCITY, 0x00, maxProfileVelocity);
	_maxProfileVelocity = maxProfileVelocity;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetProfileVelocity(uint32_t profileVelocity)
{
	if(profileVelocity > this->_maxProfileVelocity) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR;
	}
	else if(profileVelocity < 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	
	_setUint32DataFunc(CIA_402_PROFILE_VELOCITY, 0x00, profileVelocity);
	_profileVelocity = profileVelocity;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetProfileAcceleration(uint32_t profileAcceleration)
{
	if(profileAcceleration < 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	else if(profileAcceleration > _maxAcceleration) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR;
	}
	
	_setUint32DataFunc(CIA_402_PROFILE_ACCELERATION, 0x00, profileAcceleration);
	_profileAcceleration = profileAcceleration;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetProfileDeceleration(uint32_t profileDeceleration)
{
	if(profileDeceleration < 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	else if(profileDeceleration > _maxDeceleration) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR;
	}
	
	_setUint32DataFunc(CIA_402_PROFILE_DECELERATION, 0x00, profileDeceleration);
	_profileDeceleration = profileDeceleration;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetQuickStopDeceleration(uint32_t quickStopDeceleration)
{
	if(quickStopDeceleration < 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}else if(quickStopDeceleration > _maxDeceleration) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR;
	}
	
	_setUint32DataFunc(CIA_402_QUICK_STOP_DECELERATION, 0x00, quickStopDeceleration);
	_quickStopDeceleration = quickStopDeceleration;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetMaxAcceleration(uint32_t maxAcceleration)
{
	if(maxAcceleration < 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	else if(maxAcceleration < _profileAcceleration) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	
	_maxAcceleration = maxAcceleration;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetMaxDeceleration(uint32_t maxDeceleration)
{
	if(maxDeceleration < 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	else if((maxDeceleration < _profileDeceleration) || (maxDeceleration < _quickStopDeceleration)) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	
	_maxDeceleration = maxDeceleration;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetMinSwPositionLimit(int32_t minSwPositionLimit)
{
	if((minSwPositionLimit >= _maxSwPositionLimit) || (minSwPositionLimit >= _actualPosition)) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR;
	}
	
	_minSwPositionLimit = minSwPositionLimit;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetMaxSwPositionLimit(int32_t maxSwPositionLimit)
{
	if((maxSwPositionLimit <= _minSwPositionLimit) || (maxSwPositionLimit <= _actualPosition)) {
		return CAN_OPEN_ABORT_CODE_SERVICE_PARAMETER_TOO_LOW_ERROR;
	}
	
	_maxSwPositionLimit = maxSwPositionLimit;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetPositionWindow(uint32_t positionWindow)
{
	_positionWindow = positionWindow;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetPositionWindowTime(uint16_t positionWindowTime)
{
	_positionWindowTime = positionWindowTime;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetVelocityWindow(uint32_t velocityWindow)
{
	_velocityWindow = velocityWindow;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetVelocityWindowTime(uint16_t velocityWindowTime)
{
	_velocityWindowTime = velocityWindowTime;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetFollowingErrorWindow(uint32_t followingErrorWindow)
{
	if(followingErrorWindow > 2147483647) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	else if(followingErrorWindow < 1) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_followingErrorWindow = followingErrorWindow;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetPositionNotationIndex(int8_t positionNotationIndex)
{
	if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	if(positionNotationIndex != CIA_402_NOTATION_INDEX_ZERO) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_positionNotationIndex = positionNotationIndex;

	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetPositionDimensionIndex(uint8_t positionDimensionIndex)
{
	if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	if(positionDimensionIndex != CIA_402_DIMENSION_INDEX_STEPS) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_positionDimensionIndex = positionDimensionIndex;

	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetVelocityNotationIndex(int8_t velocityNotationIndex)
{
	if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	if(velocityNotationIndex != CIA_402_NOTATION_INDEX_ZERO) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_velocityNotationIndex = velocityNotationIndex;

	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetVelocityDimensionIndex(uint8_t velocityDimensionIndex)
{
	if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	if(velocityDimensionIndex != CIA_402_DIMENSION_INDEX_RPM) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}

	_velocityDimensionIndex = velocityDimensionIndex;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetAccelerationNotationIndex(int8_t accelerationNotationIndex)
{
	if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	if(accelerationNotationIndex != CIA_402_NOTATION_INDEX_ZERO) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_accelerationNotationIndex = accelerationNotationIndex;

	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetAccelerationDimensionIndex(uint8_t accelerationDimensionIndex)
{
	if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	if(accelerationDimensionIndex != CIA_402_DIMENSION_INDEX_RPM) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_accelerationDimensionIndex = accelerationDimensionIndex;

	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetHomingMethod(int8_t homingMethod)
{
	if((homingMethod == CIA_402_HOMING_METHOD_NEGATIVE_LIMIT_SWITCH) ||
	   (homingMethod == CIA_402_HOMING_METHOD_POSITIVE_LIMIT_SWITCH) ||
	   (homingMethod == CIA_402_HOMING_METHOD_HOME_SWITCH_NEGATIVE_SPEED) ||
	   (homingMethod == CIA_402_HOMING_METHOD_HOME_SWITCH_POSITIVE_SPEED) ||
	   (homingMethod == CIA_402_HOMING_METHOD_CURRENT_THRESHOLD_POSITIVE_SPEED) ||
	   (homingMethod == CIA_402_HOMING_METHOD_CURRENT_THRESHOLD_NEGATIVE_SPEED) ||
	   (homingMethod == CIA_402_HOMING_METHOD_ACTUAL_POSITION)) {
		_homingStatus = HOMING_STATUS_INIT;
		_homingMethod = homingMethod;
	}
	else {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetSwitchSearchVelocity(uint32_t switchSearchVelocity)
{
	if(switchSearchVelocity == 0) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	else if(switchSearchVelocity > _maxProfileVelocity) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR;
	}
	
	_switchSearchVelocity = switchSearchVelocity;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetZeroSearchVelocity(uint32_t zeroSearchVelocity)
{
	if(zeroSearchVelocity == 0) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	else if(zeroSearchVelocity > _maxProfileVelocity) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR;
	}
	
	_zeroSearchVelocity = zeroSearchVelocity;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetHomingAcceleration(uint32_t homingAcceleration)
{
	if(homingAcceleration == 0) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	else if(homingAcceleration > _profileAcceleration) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_HIGH_ERROR;
	}
	
	_homingAcceleration = homingAcceleration;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetHomeOffset(int32_t homeOffset)
{
	if((homeOffset > _maxSwPositionLimit) || (homeOffset < _minSwPositionLimit)) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_homeOffset = homeOffset;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetCurrentPGain(uint16_t currentPGain)
{
	_setUint16DataFunc(CIA_402_CURRENT_PARAMETER, CIA_402_CURRENT_P_GAIN, currentPGain);
	_currentPGain = currentPGain;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetCurrentIGain(uint16_t currentIGain)
{
	_setUint16DataFunc(CIA_402_CURRENT_PARAMETER, CIA_402_CURRENT_I_GAIN, currentIGain);
	_currentIGain = currentIGain;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

//pjg++181130
uint32_t CanOpenCia402::SetCurrentOffset(int16_t currentOffset)
{
	_setInt16DataFunc(CIA_402_CURRENT_PARAMETER, CIA_402_CURRENT_OFFSET, currentOffset);
	_currentOffset = currentOffset;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetVelocityPGain(uint16_t velocityPGain)
{
	_setUint16DataFunc(CIA_402_VELOCITY_PARAMETER, CIA_402_VELOCITY_P_GAIN, velocityPGain);
	_velocityPGain = velocityPGain;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetVelocityIGain(uint16_t velocityIGain)
{
	_setUint16DataFunc(CIA_402_VELOCITY_PARAMETER, CIA_402_VELOCITY_I_GAIN, velocityIGain);
	_velocityIGain = velocityIGain;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetPositionPGain(uint16_t positionPGain)
{
	_setUint16DataFunc(CIA_402_POSITION_PARAMETER, CIA_402_POSITION_P_GAIN, positionPGain);
	_positionPGain = positionPGain;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetDigitalOutput(uint32_t digitalOutput)
{
	for(int8_t i = 0; i < 32; i++) {
		if(((_digitalOutputMask >> i) & 0x1) == 0x1) {
			if(((digitalOutput >> i) & 0x1) == 0x1) {
				_digitalOutput |= (0x1 << i);
			}
			else {
				_digitalOutput &= ~(0x1 << i);
			}
		}
	}
	
	_setUint32DataFunc(CIA_402_DIGITAL_OUTPUT, CIA_402_DIGITAL_OUTPUT_PHYSICAL_OUTPUT, _digitalOutput);
		
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetDigitalOutputMask(uint32_t digitalOutputMask)
{
	_digitalOutputMask = digitalOutputMask;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}
	
uint16_t CanOpenCia402::GetAnalogIntput(uint8_t index)
{
	if(index > 4) {
		return 0;
	}
	
	return _analogInput[index];
}
