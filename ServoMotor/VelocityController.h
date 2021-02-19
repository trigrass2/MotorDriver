#pragma once

#include "CurrentController.h"
#include "SCurveProfile.h"
#include "TrapezoidalProfile.h"
#include "PositionDiff.h"

class VelocityController : public CurrentController
{
public:
	VelocityController(void);
	
	//	Load Torque
	float _Tl;				//	[Nm]			Load Torque
	
	//	Acceleration
	float _actualAcceleration;
	
	//	Velocity
	float _targetVelocity;
	float _velocityOffset;
	float _demandVelocity;
	float _prevDemandVelocity;
	float _actualVelocity;
	float _prevVelocity;
	float _estimatedVelocity;
	
	//	Motion
	int16_t _motionProfileType;
	SinusoidalProfile _sCurveProfile;
	TrapezoidalProfile _trapezoidalProfile;
	float _profileVelocity;
	float _profileAcceleration;
	float _profileDeceleration;
	float _quickStopDeceleration;
	
	//	Velocity Controller
	PiController _velocityController;
	
	
	void ResetVelocityController(void);
	
	void SetTargetVelocity(float targetVelocity);
	void SetTargetProfileVelocity(float targetProfileVelocity, float maxProfileVelocity, float acceleration, float deceleration);
	
	void CalculateVelocity(int32_t encoderPulse);
	void RunVelocityController(void);
	void RunProfileVelocityController(void);
	
	void SetMotionProfileType(int16_t motionProfileType);
	void SetProfileVelocity(float profileVelocity);
	void SetProfileAcceleration(float profileAcceleration);
	void SetProfileDeceleration(float profileDeceleration);
	void SetQuickStopDeceleration(float quickStopDeceleration);
		
	void SetVelocityPGain(float velocityPGain);
	void SetVelocityIGain(float velocityIGain);
	
	//	Position
	int32_t _targetPosition;
	int32_t _demandPosition;
	int32_t _actualPosition;
	int32_t _prevPosition;
	int32_t _prevEncoderPulse;	
	
	PositionDiff _positionDiff;
};