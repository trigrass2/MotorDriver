#pragma once

#include "math_def.h"

class PiController
{
public:
	PiController(float dt);
	
protected:
	float _dt;

	float _kp;
	float _ki;
	
	float _up;
	float _ui;
	
	float _out;
	float _maxOut;
	
public:
	void Reset(void);
	int8_t SetPGain(float kp);
	int8_t SetIGain(float ki);
	int8_t SetMaxOut(float maxOut);
	
	float Run(float ref, float feedback, float feedforward, float maxOut);
	float Run(float ref, float feedback, float feedforward);
};
