#include "MotorProperty.h"

MotorProperty::MotorProperty(void)
{
	_motorType = 0x00;
	
	_R = 1.0f;
	_Ld = 0.001f;
	_Lq = 0.001f;
	_Kt = 0.001f;
	_Ke = 0.001f;
	_Jm = 0.0f;
	_Fv = 0.0f;
	_Fc = 0.0f;
	
	_motorPhase = MOTOR_PHASE_U_V_W;

	_maximumCurrent = 1.0f;
	_ratedCurrent = 0.5f;
	_ratedTorque = 0.1f;
	
	_maximumSpeed = 10.0f;
	_maximumAcceleration = 10.0f;
	_maximumDeceleration = 10.0f;
	
	_polePair = 0;
	
	SetEncoderResolution(2000);
	
	_actualElecTheta = 0.0f;
	_prevActualElecTheta = 0.0f;
	_targetElecTheta = 0.0f;
	
	//	UVW		:	110		100		101		001		011		010		6	4	5	1	3	2
	//	UWV 	:	101		100		110		010		011		001		5	4	6	2	3	1
	//	VUW		:	110		010		011		001		101		100		6	2	3	1	5	4
	//	VWU		:	101		001		011		010		110		100		5	1	3	2	6	4
	//	WUV		:	011		010		110		100		101		001		3	2	6	4	5	1
	//	WVU		:	011		001		101		100		110		010		3	1	5	4	6	2
	//	~UVW	:	001		011		010		110		100		101		1	3	2	6	4	5
	//	~UWV 	:	010		011		001		101		100		110		2	3	1	5	4	6
	//	~VUW	:	001		101		100		110		010		011		1	5	4	6	2	3
	//	~VWU	:	010		110		100		101		001		011		2	6	4	5	1	3
	//	~WUV	:	100		101		001		011		010		110		4	5	1	3	2	6
	//	~WVU	:	100		110		010		011		001		101		4	6	2	3	1	5
	_hallSensorPatternBuf[0][0] = 6;	_hallSensorPatternBuf[0][1] = 4;	_hallSensorPatternBuf[0][2] = 5;	_hallSensorPatternBuf[0][3] = 1;	_hallSensorPatternBuf[0][4] = 3;	_hallSensorPatternBuf[0][5] = 2;
	_hallSensorPatternBuf[1][0] = 5;	_hallSensorPatternBuf[1][1] = 4;	_hallSensorPatternBuf[1][2] = 6;	_hallSensorPatternBuf[1][3] = 2;	_hallSensorPatternBuf[1][4] = 3;	_hallSensorPatternBuf[1][5] = 1;
	_hallSensorPatternBuf[2][0] = 6;	_hallSensorPatternBuf[2][1] = 2;	_hallSensorPatternBuf[2][2] = 3;	_hallSensorPatternBuf[2][3] = 1;	_hallSensorPatternBuf[2][4] = 5;	_hallSensorPatternBuf[2][5] = 4;
	_hallSensorPatternBuf[3][0] = 5;	_hallSensorPatternBuf[3][1] = 1;	_hallSensorPatternBuf[3][2] = 3;	_hallSensorPatternBuf[3][3] = 2;	_hallSensorPatternBuf[3][4] = 6;	_hallSensorPatternBuf[3][5] = 4;
	_hallSensorPatternBuf[4][0] = 3;	_hallSensorPatternBuf[4][1] = 2;	_hallSensorPatternBuf[4][2] = 6;	_hallSensorPatternBuf[4][3] = 4;	_hallSensorPatternBuf[4][4] = 5;	_hallSensorPatternBuf[4][5] = 1;
	_hallSensorPatternBuf[5][0] = 3;	_hallSensorPatternBuf[5][1] = 1;	_hallSensorPatternBuf[5][2] = 5;	_hallSensorPatternBuf[5][3] = 4;	_hallSensorPatternBuf[5][4] = 6;	_hallSensorPatternBuf[5][5] = 2;
	_hallSensorPatternBuf[6][0] = 1;	_hallSensorPatternBuf[6][1] = 3;	_hallSensorPatternBuf[6][2] = 2;	_hallSensorPatternBuf[6][3] = 6;	_hallSensorPatternBuf[6][4] = 4;	_hallSensorPatternBuf[6][5] = 5;
	_hallSensorPatternBuf[7][0] = 2;	_hallSensorPatternBuf[7][1] = 3;	_hallSensorPatternBuf[7][2] = 1;	_hallSensorPatternBuf[7][3] = 5;	_hallSensorPatternBuf[7][4] = 4;	_hallSensorPatternBuf[7][5] = 6;
	_hallSensorPatternBuf[8][0] = 1;	_hallSensorPatternBuf[8][1] = 5;	_hallSensorPatternBuf[8][2] = 4;	_hallSensorPatternBuf[8][3] = 6;	_hallSensorPatternBuf[8][4] = 2;	_hallSensorPatternBuf[8][5] = 3;
	_hallSensorPatternBuf[9][0] = 2;	_hallSensorPatternBuf[9][1] = 6;	_hallSensorPatternBuf[9][2] = 4;	_hallSensorPatternBuf[9][3] = 5;	_hallSensorPatternBuf[9][4] = 1;	_hallSensorPatternBuf[9][5] = 3;
	_hallSensorPatternBuf[10][0] = 4;	_hallSensorPatternBuf[10][1] = 5;	_hallSensorPatternBuf[10][2] = 1;	_hallSensorPatternBuf[10][3] = 3;	_hallSensorPatternBuf[10][4] = 2;	_hallSensorPatternBuf[10][5] = 6;
	_hallSensorPatternBuf[11][0] = 4;	_hallSensorPatternBuf[11][1] = 6;	_hallSensorPatternBuf[11][2] = 2;	_hallSensorPatternBuf[11][3] = 3;	_hallSensorPatternBuf[10][4] = 1;	_hallSensorPatternBuf[11][5] = 5;
	
	ResetHallSensorPattern();
	
}

int8_t MotorProperty::SetRatedCurrent(float ratedCurrent)
{
	if(ratedCurrent <= 0.0f) {
		return -1;
	}
	
	_ratedCurrent = ratedCurrent;
	_Kt = _ratedTorque / _ratedCurrent;
	
	return 0;
}

int8_t MotorProperty::SetRatedTorque(float ratedTorque)
{
	if(ratedTorque <= 0.0f) {
		return -1;
	}
	
	_ratedTorque = ratedTorque;
	_Kt = _ratedTorque / _ratedCurrent;
	
	return 0;
}

int8_t MotorProperty::SetMaximumSpeed(float maximumSpeed)
{
	if(maximumSpeed <= 0.0f) {
		return -1;
	}
	
	_maximumSpeed = maximumSpeed;
	
	return 0;
}

int8_t MotorProperty::SetPolePair(uint8_t polePair)
{
	if(polePair < 1) {
		return -1;
	}
	
	_polePair = polePair;
	
	return 0;
}

int8_t MotorProperty::SetEncoderResolution(int32_t encoderResolution)
{
	if(encoderResolution < 6) {
		return -1;
	}
	
	_encoderResolution = encoderResolution;
	
	if(_motorType == MOTOR_TYPE_LINEAR) {
		_pulse2deg = 1.0f / (float)_encoderResolution;
		_deg2pulse = (float)_encoderResolution / 1.0f;
		_pulse2rad = 1.0f / (float)_encoderResolution;
		_rad2pulse = (float)_encoderResolution / 1.0f;
	}
	else {
		_pulse2deg = 360.0f / (float)_encoderResolution;
		_deg2pulse = (float)_encoderResolution / 360.0f;
		_pulse2rad = (2.0f * M_PI) / (float)_encoderResolution;
		_rad2pulse = (float)_encoderResolution / (2.0f * M_PI);
	}
	
	return 0;
}

int8_t MotorProperty::SetMotorPhase(uint8_t motorPhase)
{
	if(motorPhase >= 6) {
		return -1;
	}
	
	_motorPhase = motorPhase;
	
	return 0;
}

void MotorProperty::ResetHallSensorPattern(void)
{
	_hallSensorPattern[0] = 0;
	_hallSensorPattern[1] = 0;
	_hallSensorPattern[2] = 0;
	_hallSensorPattern[3] = 0;
	_hallSensorPattern[4] = 0;
	_hallSensorPattern[5] = 0;
}

int8_t MotorProperty::SearchHallSensorPattern(uint8_t hallSensorPattern, float angle)
{
	int8_t i = 0;
	
	if((angle > -1.0f) && (angle < 1.0f)) {
		_hallSensorPattern[0] = hallSensorPattern;
	}
	else if((angle > 59.0f) && (angle < 61.0f)) {
		_hallSensorPattern[1] = hallSensorPattern;
	}
	else if((angle > 119.0f) && (angle < 121.0f)) {
		_hallSensorPattern[2] = hallSensorPattern;
	}
	else if((angle > 179.0f) && (angle < 181.0f)) {
		_hallSensorPattern[3] = hallSensorPattern;
	}
	else if((angle > 239.0f) && (angle < 241.0f)) {
		_hallSensorPattern[4] = hallSensorPattern;
	}
	else if((angle > 299.0f) && (angle < 301.0f)) {
		_hallSensorPattern[5] = hallSensorPattern;
	}
	
	for(i = 0; i < 6; i++) {
		if(_hallSensorPattern[i] == 0) {
			return 0;
		}
	}
	
	for(int8_t i = 0; i < 12; i++) {
		if(	(_hallSensorPattern[0] == _hallSensorPatternBuf[i][0]) &&
		   	(_hallSensorPattern[1] == _hallSensorPatternBuf[i][1]) &&
			(_hallSensorPattern[2] == _hallSensorPatternBuf[i][2]) &&
			(_hallSensorPattern[3] == _hallSensorPatternBuf[i][3]) &&
			(_hallSensorPattern[4] == _hallSensorPatternBuf[i][4]) &&
			(_hallSensorPattern[5] == _hallSensorPatternBuf[i][5])) {
			//_motorPhase = 9;
			return 1;
		}
	}
	
	return -1;
}

