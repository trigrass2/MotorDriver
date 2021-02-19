#include "utils.h"
#include "CanOpenCia402.h"

uint32_t CanOpenCia402::SetResistance(uint16_t resistance)
{
	if(resistance == 0) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	
	_setUint16DataFunc(KITECH_CIA_402_RESISTANCE, 0x00, resistance);
	_resistance = resistance;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetQAxisInductance(uint32_t qAxisInductance)
{
	if(qAxisInductance == 0) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	
	_setUint32DataFunc(KITECH_CIA_402_Q_AXIS_INDUCTANCE, 0x00, qAxisInductance);
	_qAxisInductance = qAxisInductance;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetDAxisInductance(uint32_t dAxisInductance)
{
	if(dAxisInductance == 0) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	
	_setUint32DataFunc(KITECH_CIA_402_D_AXIS_INDUCTANCE, 0x00, dAxisInductance);
	_dAxisInductance = dAxisInductance;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetTorqueConstant(uint32_t torqueConstant)
{
	if(torqueConstant == 0) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	
	_setUint32DataFunc(KITECH_CIA_402_TORQUE_CONSTANT, 0x00, torqueConstant);
	_torqueConstant = torqueConstant;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetBackEmfConstant(uint32_t backEmfConstant)
{
	if(backEmfConstant == 0) {
		return CAN_OPEN_ABORT_CODE_VALUE_TOO_LOW_ERROR;
	}
	
	_setUint32DataFunc(KITECH_CIA_402_BACK_EMF_CONSTANT, 0x00, backEmfConstant);
	_backEmfConstant = backEmfConstant;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetSystemInertia(uint32_t systemInertia)
{
	_setUint32DataFunc(KITECH_CIA_402_SYSTEM_INERTIA, 0x00, systemInertia);
	_systemInertia = systemInertia;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetCoulombFriction(uint32_t coulombFriction)
{
	_setUint32DataFunc(KITECH_CIA_402_COULOMB_FRICTION, 0x00, coulombFriction);
	_coulombFriction = coulombFriction;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetViscosFriction(uint32_t viscosFriction)
{
	_setUint32DataFunc(KITECH_CIA_402_VISCOS_FRICTION, 0x00, viscosFriction);
	_viscosFriction = viscosFriction;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetElectricAngleOffset(int16_t electricAngleOffset)
{
	if((electricAngleOffset > 3600) || (electricAngleOffset < -3600)) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_setInt16DataFunc(KITECH_CIA_402_ELECTRIC_ANGLE_OFFSET, 0x00, electricAngleOffset);
	_electricAngleOffset = electricAngleOffset;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetMotorPhase(uint8_t motorPhase)
{
	if(motorPhase >= 6) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	_setInt8DataFunc(KITECH_CIA_402_MOTOR_PHASE, 0x00, motorPhase);
	_motorPhase = motorPhase;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetPolePair(uint8_t polePair)
{
	_setUint8DataFunc(KITECH_CIA_402_HALL_SENSOR_POLE_PAIR, 0x00, polePair);
	_polePairs = polePair;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetDAxisVoltage(int32_t dAxisVoltage)
{
	if(STATUS_WORD_STATE != CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	else if(_modesOfOperationDisplay != CIA_402_VOLTAGE_MODE) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	if((dAxisVoltage > (int32_t)_dcLinkVoltage) || (dAxisVoltage < -(int32_t)_dcLinkVoltage)) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	if(_setTargetVoltage == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
	_setTargetVoltage(dAxisVoltage, _qAxisVoltage);

	_dAxisVoltage = dAxisVoltage;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetQAxisVoltage(int32_t qAxisVoltage)
{
	if(STATUS_WORD_STATE != CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	else if(_modesOfOperationDisplay != CIA_402_VOLTAGE_MODE) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	if((qAxisVoltage > (int32_t)_dcLinkVoltage) || (qAxisVoltage < -(int32_t)_dcLinkVoltage)) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	if(_setTargetVoltage == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
	_setTargetVoltage(_dAxisVoltage, qAxisVoltage);
	
	_qAxisVoltage = qAxisVoltage;
	
	_bufIndex = 0;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetTargetCurrent(int32_t targetCurrent)
{
	if(STATUS_WORD_STATE != CIA_402_STATUS_OPERATION_ENABLED) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	else if(_modesOfOperationDisplay != CIA_402_CURRENT_MODE) {
		return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	else if(targetCurrent > (int32_t)_ratedCurrent) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	else if(targetCurrent < (int32_t)-_ratedCurrent) {
		return CAN_OPEN_ABORT_CODE_VALUE_RANGE_ERROR;
	}
	
	if(_setTargetCurrent == NULL) return CAN_OPEN_ABORT_CODE_LOCAL_CONTROL_ERROR;
	_setTargetCurrent(targetCurrent);
	
	_targetCurrent = targetCurrent;
	
	_bufIndex = 0;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetLoadTorqueThreshold(int32_t loadTorqueThreshold)
{
	_loadTorqueThreshold = loadTorqueThreshold;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetVelocityFollowingError(uint32_t velocityFollowingError)
{
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetVelocityAutoTuningAcceleration(uint8_t velocityAutoTuningAcceleration){
	_velocityAutoTuningAcceleration = velocityAutoTuningAcceleration;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetVelocityAutoTuningVelocity(uint8_t velocityAutoTuningVelocity)
{
	_velocityAutoTuningVelocity = velocityAutoTuningVelocity;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetVelocityAutoTuningPosition(int32_t velocityAutoTuningPosition)
{
	_velocityAutoTuningPosition = velocityAutoTuningPosition;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetVelocityControllerBandwidth(uint16_t velocityControllerBandwidth)
{
	_velocityControllerBandwidth = velocityControllerBandwidth;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetPcpModeHomePosition(int32_t pcpModeHomePosition)
{
	if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
		if(_pcpModeStatus != PCP_MODE_STATUS_READY)	return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}

	_pcpModeHomePosition = pcpModeHomePosition;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}
	
uint32_t CanOpenCia402::SetPcpModeTargetPosition(int32_t pcpModeTargetPosition)
{
	if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
		if(_pcpModeStatus != PCP_MODE_STATUS_READY)	return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
		
	_pcpModeTargetPosition = pcpModeTargetPosition;

	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetPcpModeTargetCurrent(int32_t pcpModeTargetCurrent)
{
	if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
		if(_pcpModeStatus != PCP_MODE_STATUS_READY)	return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	_pcpModeTargetCurrent = pcpModeTargetCurrent;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}

uint32_t CanOpenCia402::SetPcpModeCurrentModeDuration(uint16_t pcpModeCurrentModeDuration)
{
	if(STATUS_WORD_STATE == CIA_402_STATUS_OPERATION_ENABLED) {
		if(_pcpModeStatus != PCP_MODE_STATUS_READY)	return CAN_OPEN_ABORT_CODE_WRONG_DEVICE_STATE;
	}
	
	_pcpModeCurrentModeDuration = pcpModeCurrentModeDuration;
	
	return CAN_OPEN_ABORT_CODE_NO_ERROR;
}
