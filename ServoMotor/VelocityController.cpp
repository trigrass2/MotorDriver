#include "VelocityController.h"
#include "peripheral.h"

VelocityController::VelocityController(void)
: _velocityController(VELOCITY_CONTROLLER_PERIOD), _sCurveProfile((long)VELOCITY_CONTROLLER_FREQ), _trapezoidalProfile((long)VELOCITY_CONTROLLER_FREQ), _positionDiff((int)VELOCITY_CONTROLLER_FREQ)
{
	//	Load torque
	_Tl = 0.0f;
	
	//	Acceleration
	_actualAcceleration = 0.0f;
	
	//	Velocity
	_velocityOffset = 0.0f;
	_actualVelocity = 0.0f;
	_prevVelocity = 0.0f;
	_estimatedVelocity = 0.0f;
	
	//	Motion
	_motionProfileType = 0;
	
	_profileVelocity = 0.0f;
	_profileAcceleration = 0.0f;
	_profileDeceleration = 0.0f;
	_quickStopDeceleration = 0.0f;
	
	
	//	Position
	_targetPosition = 0;
	_demandPosition = 0;
	_actualPosition = 0;
	_prevPosition = 0;
	_prevEncoderPulse = 0;
	
	ResetVelocityController();
}

void VelocityController::ResetVelocityController(void)
{
	//	Velocity
	_targetVelocity = 0.0f;
	_demandVelocity = 0.0f;
	_prevDemandVelocity = 0.0f;
	
	//	Motion
	_trapezoidalProfile.Reset();
	_sCurveProfile.Reset();
	
	//	Velocity Controller
	_velocityController.Reset();
}

void VelocityController::SetTargetVelocity(float targetVelocity)
{
	if(targetVelocity > _maximumSpeed)			_targetVelocity = _maximumSpeed;
	else if(targetVelocity < -_maximumSpeed)	_targetVelocity = -_maximumSpeed;
	else										_targetVelocity = targetVelocity;
	
	_prevDemandVelocity = _actualVelocity;
	_demandVelocity = _targetVelocity;
}

void VelocityController::SetTargetProfileVelocity(float targetProfileVelocity, float maxProfileVelocity, float acceleration, float deceleration)
{
	if(_motionProfileType == 0) {
		//	Trapezoidal Profile
		_trapezoidalProfile.Reset();
		_trapezoidalProfile.SetParam(_pulse2rad, _rad2pulse);
		_trapezoidalProfile.InitVelocityProfile(targetProfileVelocity, _actualVelocity, maxProfileVelocity, acceleration, deceleration);
	}
	else if(_motionProfileType == 1) {
		//	S-Curve Profile
		_sCurveProfile.Reset();
		if(_motorType == MOTOR_TYPE_LINEAR) {
			_sCurveProfile.SetParam(_pulse2rad, _rad2pulse, 1.0f);
		}
		else {
			_sCurveProfile.SetParam(_pulse2rad, _rad2pulse, M_PI);
		}
		_sCurveProfile.InitVelocityProfile(targetProfileVelocity, _actualVelocity, maxProfileVelocity, acceleration, deceleration, acceleration*2.0f);
	}
	else {
		_trapezoidalProfile.Reset();
		_sCurveProfile.Reset();
		return;
	}
	
	_targetVelocity = targetProfileVelocity;
	_prevDemandVelocity = _actualVelocity;
	_demandVelocity = _actualVelocity;
}

void VelocityController::CalculateVelocity(int32_t encoderPulse)
{
	_actualPosition += (encoderPulse - _prevEncoderPulse);
	_prevEncoderPulse = encoderPulse;
	
	_actualVelocity = (_actualPosition - _prevPosition) * VELOCITY_CONTROLLER_FREQ * _pulse2rad;
	_prevPosition = _actualPosition;
	
	/*
	long velocity, acceleration;
	_positionDiff.VelocityAcceleration(velocity, acceleration, _actualPosition, 4);
	_estimatedVelocity = (float)velocity * _pulse2rad;
	if(abs(_estimatedVelocity - _actualVelocity) < (_pulse2rad*VELOCITY_CONTROLLER_FREQ)) {
		_actualVelocity = _estimatedVelocity;
	}
	*/
		
	_actualAcceleration = (_actualVelocity - _prevVelocity) * VELOCITY_CONTROLLER_FREQ;
	_prevVelocity = _actualVelocity;
	
	_backEmfVelocity = _actualVelocity;
}

void VelocityController::RunVelocityController(void)
{
	float demandVelocity;
	
	//	Check the acceleration limit
	if(_demandVelocity > _prevDemandVelocity) {
		if((_demandVelocity - _prevDemandVelocity) > _maximumAcceleration*VELOCITY_CONTROLLER_PERIOD) {
			demandVelocity = _prevDemandVelocity + _maximumAcceleration*VELOCITY_CONTROLLER_PERIOD;
		}
		else {
			demandVelocity = _demandVelocity;
		}
	}
	else if(_demandVelocity < _prevDemandVelocity) {
		if((_demandVelocity - _prevDemandVelocity) < -_maximumAcceleration*VELOCITY_CONTROLLER_PERIOD) {
			demandVelocity = _prevDemandVelocity - _maximumAcceleration*VELOCITY_CONTROLLER_PERIOD;
		}
		else {
			demandVelocity = _demandVelocity;
		}
	}
	else {
		demandVelocity = _demandVelocity;
	}
	
	//	Calculate the Friction
	float feedForward = _Fv*demandVelocity;
	if(_Kt > 0.0f)	feedForward += _Tl / _Kt;

	if(_motorType == MOTOR_TYPE_LINEAR) {
		//	Threshod : 0.01m/s
		if(demandVelocity > 0.01f)			feedForward += _Fc;
		else if(demandVelocity < -0.01f)	feedForward += -_Fc;
	}
	else {
		//	Threshod : 1.047197512rad/s -> 10RPM
		if(demandVelocity > 1.04719512f)			feedForward += _Fc;
		else if(demandVelocity < -1.04719512f)		feedForward += -_Fc;
	}
	
	_dAxisTargetCurrent = 0.0f;
	_qAxisTargetCurrent = _velocityController.Run(demandVelocity, _actualVelocity, feedForward, _maximumCurrent);
	
	_prevDemandVelocity = demandVelocity;
}

void VelocityController::RunProfileVelocityController(void)
{
	float acceleration, velocity;
	
	if(_motionProfileType == 0) {
		//	Trapezoidal Profile
		if(_trapezoidalProfile.VelocityProfile(velocity, acceleration) == true) {
			_demandVelocity = velocity;
		}
		else {
			_demandVelocity = _targetVelocity;
		}
	}
	else if(_motionProfileType == 1) {
		//	S-Curve Profile
		if(_sCurveProfile.VelocityProfile(velocity, acceleration) == true) {
			_demandVelocity = velocity;
		}
		else {
			_demandVelocity = _targetVelocity;
		}
	}
	else {
		_demandVelocity = _targetVelocity;
	}
	
	float feedForward = _Fv*_demandVelocity;
	
	if(_Kt > 0.0f)	feedForward += _Tl / _Kt;

	if(_motorType == MOTOR_TYPE_LINEAR) {
		//	Threshod : 0.01m/s
		if(_demandVelocity > 0.01f)			feedForward += _Fc;
		else if(_demandVelocity < -0.01f)	feedForward += -_Fc;
	}
	else {
		//	Threshod : 1.047197512rad/s -> 10RPM
		if(_demandVelocity > 1.04719512f)			feedForward += _Fc;
		else if(_demandVelocity < -1.04719512f)		feedForward += -_Fc;
	}
	
	_dAxisTargetCurrent = 0.0f;
	_qAxisTargetCurrent = _velocityController.Run(_demandVelocity, _actualVelocity, feedForward, _maximumCurrent);
	
	_prevDemandVelocity = _demandVelocity;
}

void VelocityController::SetMotionProfileType(int16_t motionProfileType)
{
	_motionProfileType = motionProfileType;
}

void VelocityController::SetProfileVelocity(float profileVelocity)
{
	_profileVelocity = profileVelocity;
}

void VelocityController::SetProfileAcceleration(float profileAcceleration)
{
	_profileAcceleration = profileAcceleration;
}

void VelocityController::SetProfileDeceleration(float profileDeceleration)
{
	_profileDeceleration = profileDeceleration;
}

void VelocityController::SetQuickStopDeceleration(float quickStopDeceleration)
{
	_quickStopDeceleration = quickStopDeceleration;
}

void VelocityController::SetVelocityPGain(float velocityPGain)
{
	_velocityController.SetPGain(velocityPGain);
}

void VelocityController::SetVelocityIGain(float velocityIGain)
{
	_velocityController.SetIGain(velocityIGain);
}




