#include "PidController.h"

PiController::PiController(float dt)
: _dt(dt)
{
	_kp = 0.0f;
	_ki = 0.0f;
	
	_up = 0.0f;
	_ui = 0.0f;
	
	_out = 0.0f;
	_maxOut = 0.0f;
}

void PiController::Reset(void)
{
	_ui = 0.0f;
	_out = 0.0f;
}

int8_t PiController::SetPGain(float kp)
{
	if(kp < 0.0f) {
		return -1;
	}
	
	_kp = kp;
	
	return 0;
}

int8_t PiController::SetIGain(float ki)
{
	if(ki < 0.0f) {
		return -1;
	}
	
	_ki = ki;
	
	return 0;
}

int8_t PiController::SetMaxOut(float maxOut)
{
	if(maxOut <= 0.0f) {
		return -1;
	}
	
	_maxOut = maxOut;
	
	return 0;
}

float PiController::Run(float ref, float feedback, float feedforward, float maxOut)
{
	float error = ref - feedback;
	
	_maxOut = maxOut;
	
	_up = error * _kp;
	_ui += (error * _ki * _dt);
	
	if(_up > _maxOut)				_up = _maxOut;
	else if(_up < -_maxOut)			_up = -_maxOut;
	
	if(_ui > _maxOut)				_ui = _maxOut;
	else if(_ui < -_maxOut)			_ui = -_maxOut;
	
	if(feedforward > _maxOut)		feedforward = _maxOut;
	else if(feedforward < -_maxOut)	feedforward = -_maxOut;
	
	_out = _up + _ui + feedforward;
	
	if(_out > _maxOut)			_out = _maxOut;
	else if(_out < -_maxOut)	_out = -_maxOut;
	
	return _out;
}

float PiController::Run(float ref, float feedback, float feedforward)
{
	float error = ref - feedback;
	
	_up = error * _kp;
	_ui += (error * _ki * _dt);
	
	if(_up > _maxOut)				_up = _maxOut;
	else if(_up < -_maxOut)			_up = -_maxOut;
	
	if(_ui > _maxOut)				_ui = _maxOut;
	else if(_ui < -_maxOut)			_ui = -_maxOut;
	
	if(feedforward > _maxOut)		feedforward = _maxOut;
	else if(feedforward < -_maxOut)	feedforward = -_maxOut;
	
	_out = _up + _ui + feedforward;
	
	if(_out > _maxOut)			_out = _maxOut;
	else if(_out < -_maxOut)	_out = -_maxOut;
	
	return _out;
}
