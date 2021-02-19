#pragma once

#include "MotorProperty.h"
#include "PidController.h"
#include "MovingAverage.h"

#define CUR_ERR_HALL_PATTERN
#define CUR_ERR_HALL_LINE
#define CUR_ERR_ENC_LINE

class CurrentController : public MotorProperty
{
public:
	CurrentController(void);
	
	//	PWM
	int16_t _pwm1;
	int16_t _pwm2;
	int16_t _pwm3;
	
	//	Voltage
	float _uVoltage;
	float _vVoltage;
	float _wVoltage;
	
	float _dAxisVoltage;
	float _qAxisVoltage;
	
	float _minActualVoltage;
	float _maxActualVoltage;
	
	float _actualVoltage;
	float _maxOutputVoltage;
	
	uint16_t _nUnderVoltageCnt; //pjg++>180202
	
	void ResetVoltageController(void);
	void SetDAxisVoltage(float dAxisVoltage);
	void SetQAxisVoltage(float qAxisVoltage);
	void ControlVoltageDC(void);
	void ControlVoltageBLDC(void);
	void ControlVoltagePMSM(void);
	
	//	Current
	float _uCurrent;
	float _vCurrent;
	float _wCurrent;
	
	int16_t _nUOverCurrent;
	int16_t _nVOverCurrent;
	int16_t _nWOverCurrent;
	int16_t _nOverCurrent;
	//pjg++>180126
	int16_t _nMaxCurrent;
	uint16_t _nMaxCurrentOccurCnt;
	//pjg<++180126
	
	float _dAxisTargetCurrent;
	float _qAxisTargetCurrent;
	float _currentOffset;
	float _currentOffsetable; //pjg++181130

	
	float _dAxisActualCurrent;
	float _qAxisActualCurrent;
	float _dAxisPrevActualCurrent;
	float _qAxisPrevActualCurrent;
	float _dAxisAveragedCurrent;
	float _qAxisAveragedCurrent;
	float _actualCurrent;
	
	void ResetCurrentController(void);
	void SetDAxisTargetCurrent(float dAxisTargetCurrent);	//	A
	void SetQAxisTargetCurrent(float qAxisTargetCurrent);	//	A
	
	void CalculateCurrentDC(void);
	void CalculateCurrentBLDC(void);
	void CalculateCurrentPMSM(void);
	int8_t CheckOverCurrent(void);
	
	void RunCurrentControllerDC(void);
	void RunCurrentControllerBLDC(void);
	void RunCurrentControllerPMSM(void);
	
	//	Hall Sensor
	uint8_t _hallStatus;
	uint8_t _prevHallStatus;
	uint8_t _hallSector;
	//uint32_t _hallSameCnt; //pjg++180827
	//uint32_t _hallErrCnt; //pjg++180827
	int8_t _hallDir; //pjg++181016
	uint32_t _hallDirErrCnt; //pjg++181016
	int32_t _hallCnt; //pjg++181101
	//int32_t _hall2EncCnt; //pjg++181101
	int32_t _hallPrevPosition;
	int32_t _hall2EncGap; //pjg++181101
	//int32_t _hall2EncMaxGap; //pjg++181101
	//int32_t _hall2EncMinGap; //pjg++181101
	//int32_t _hallAngle; //pjg++181101
	//int32_t _encAngle; //pjg++181101
	
	//	Electric Angle(theta)
	uint8_t _focStatus;
	int32_t _elecActualPosition;
	int32_t _elecPrevPosition;
	//uint32_t _encSameCnt; //pjg++180827
	float _elecInitAngle;
	float _elecAngleOffset;							void SetElectricAngleOffset(float elecAngleOffset);		//	degree
	float _resolverAngle;

	float _elecTheta;
	float _prevElecTheta;
	float _elecSinTheta;
	float _elecCosTheta;
	float _elecThetaError;
	float _prevElecThetaError;
	
	void ResetElecTheta(void);
	int CalculateElecTheta(int32_t encoderPulse);
	void CalculateElecThetaError(void);
	int ChkHallSensorPattern(); //pjg++181015

	//	Current Controller
	PiController _dAxisCurrentController;
	PiController _qAxisCurrentController;
	float _backEmfVelocity;
	
	void SetCurrentPGain(float currentPGain);
	void SetCurrentIGain(float currentIGain);
	void SetCurrentOffset(float currentOffset); //pjg++181130
};
