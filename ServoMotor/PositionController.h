#pragma once

#include "VelocityController.h"

class PositionController : public VelocityController
{
public:
	PositionController(void);
	
	float _positionPGain;
	
	void ResetPositionController(void);
		
	void SetTargetPosition(int32_t targetPosition);
	void SetTargetProfilePosition(int32_t targetPosition);
	void SetTargetProfilePosition(int32_t targetPosition, float maxVelocity, float acceleration, float deceleration);
	
	void RunPositionController(void);
	void RunProfilePositionController(void);
	
	void SetPositionPGain(float positionPGain);
};