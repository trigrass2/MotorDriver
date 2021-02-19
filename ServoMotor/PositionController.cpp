#include "peripheral.h"
#include "PositionController.h"

PositionController::PositionController(void)
{
	_positionPGain = 0.0f;
	
	ResetPositionController();
}

void PositionController::ResetPositionController(void)
{
	_demandPosition = _targetPosition = _actualPosition;
}

void PositionController::SetTargetPosition(int32_t targetPosition)
{
	_demandVelocity = _actualVelocity;
	_demandPosition = _targetPosition = targetPosition;
}

void PositionController::SetTargetProfilePosition(int32_t targetPosition)
{
	_demandVelocity = _actualVelocity;
	
	if(_motionProfileType == 0) {
		//	Trapezoidal Profile
		_trapezoidalProfile.Reset();
		_trapezoidalProfile.SetParam(_pulse2rad, _rad2pulse);
		_trapezoidalProfile.InitPositionProfile((targetPosition - _actualPosition), _actualPosition, _actualVelocity, _profileVelocity, _profileAcceleration, _profileDeceleration);
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
		
		_sCurveProfile.InitPositionProfile((targetPosition - _actualPosition), _actualPosition, _actualVelocity, _profileVelocity, _profileAcceleration, _profileDeceleration, _profileAcceleration*2.0f);
	}
	else {
		_trapezoidalProfile.Reset();
		_sCurveProfile.Reset();
		return;
	}

	_targetPosition = targetPosition;
}

void PositionController::SetTargetProfilePosition(int32_t targetPosition, float maxVelocity, float acceleration, float deceleration)
{
	_demandVelocity = _actualVelocity;
	
	if(_motionProfileType == 0) {
		//	Trapezoidal Profile
		_trapezoidalProfile.Reset();
		_trapezoidalProfile.SetParam(_pulse2rad, _rad2pulse);
		_trapezoidalProfile.InitPositionProfile((targetPosition - _actualPosition), _actualPosition, _actualVelocity, maxVelocity, acceleration, deceleration);
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
		
		_sCurveProfile.InitPositionProfile((targetPosition - _actualPosition), _actualPosition, _actualVelocity, maxVelocity, acceleration, deceleration, acceleration*2.0f);
	}
	else {
		_trapezoidalProfile.Reset();
		_sCurveProfile.Reset();
		return;
	}

	_targetPosition = targetPosition;
}

void PositionController::RunPositionController(void)
{
	float targetVelocity = (float)(_demandPosition - _actualPosition)*_pulse2rad*_positionPGain;
	
	//	Check the velocity limit
	if(targetVelocity > _maximumSpeed)			targetVelocity = _maximumSpeed;
	else if(targetVelocity < -_maximumSpeed)	targetVelocity = -_maximumSpeed;
	
	_demandVelocity = targetVelocity;
}

void PositionController::RunProfilePositionController(void)
{
	float targetVelocity;
	float acceleration, velocity;
	float positionError = (float)(_demandPosition - _actualPosition)*_pulse2rad;
	float maximumSpeed = _maximumSpeed;
	
	if(_motionProfileType == 0) {
		//	Trapezoidal Profile
		if(_trapezoidalProfile.PositionProfile(_demandPosition, velocity, acceleration) == false) {
			_demandPosition = _targetPosition;
			targetVelocity = (float)(_demandPosition - _actualPosition)*_pulse2rad*_positionPGain;
		}
		else {
			targetVelocity = velocity + positionError*_positionPGain;
			maximumSpeed *= 1.1f;
		}
	}
	else if(_motionProfileType == 1) {
		//	S-Curve Profile
		if(_sCurveProfile.PositionProfile(_demandPosition, velocity, acceleration) == false) {
			_demandPosition = _targetPosition;
			targetVelocity = (float)(_demandPosition - _actualPosition)*_pulse2rad*_positionPGain;
		}
		else {
			targetVelocity = velocity + positionError*_positionPGain;
		}
	}
	else {
		targetVelocity = positionError*_positionPGain;
	}
	
	//	Check the velocity limit
	if(targetVelocity > maximumSpeed)			targetVelocity = maximumSpeed;
	else if(targetVelocity < -maximumSpeed)		targetVelocity = -maximumSpeed;
	
	_demandVelocity = targetVelocity;
}

void PositionController::SetPositionPGain(float positionPGain)
{
	_positionPGain = positionPGain;
}