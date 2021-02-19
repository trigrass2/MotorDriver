#pragma once

#include "MotorProperty.h"
#include "PidController.h"
#include "MovingAverage.h"

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
	
	float _dAxisTargetCurrent;
	float _qAxisTargetCurrent;
	float _currentOffset;
	
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
	
	//	Electric Angle(theta)
	uint8_t _focStatus;
	int32_t _elecActualPosition;
	int32_t _elecPrevPosition;
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
	void CalculateElecTheta(int32_t encoderPulse);
	void CalculateElecThetaError(void);
	
	//	Current Controller
	PiController _dAxisCurrentController;
	PiController _qAxisCurrentController;
	float _backEmfVelocity;
	
	void SetCurrentPGain(float currentPGain);
	void SetCurrentIGain(float currentIGain);
};