#pragma once

#include "CanOpen.h"
#include "MotorProperty.h"
#include "PositionController.h"
#include "KalmanFilter.h"
#include "MovingAverage.h"

#define	SC_VOLTAGE_CTRL_BIT				0x0001
#define	SC_CURRENT_CTRL_BIT				0x0002
#define	SC_VELOCITY_CTRL_BIT			0x0004
#define	SC_POSITION_CTRL_BIT			0x0008
#define	SC_PROFILE_MODE_BIT				0x0010

#define	SC_VOLTAGE_AUTO_TUNING_BIT		0x0100
#define	SC_CURRENT_AUTO_TUNING_BIT		0x0200
#define	SC_VELOCITY_AUTO_TUNING_BIT		0x0400
#define	SC_POSITION_AUTO_TUNING_BIT		0x0800
#define	SC_ERROR_BIT					0x8000

#define	SC_DISABLE_MODE					(0x0000)
#define	SC_VOLTAGE_MODE					(SC_VOLTAGE_CTRL_BIT)
#define	SC_VOLTAGE_AUTO_TUNING_MODE		(SC_VOLTAGE_CTRL_BIT | SC_VOLTAGE_AUTO_TUNING_BIT)
#define	SC_CURRENT_MODE					(SC_VOLTAGE_CTRL_BIT | SC_CURRENT_CTRL_BIT)
#define	SC_CURRENT_AUTO_TUNING_MODE		(SC_VOLTAGE_CTRL_BIT | SC_CURRENT_CTRL_BIT | SC_CURRENT_AUTO_TUNING_BIT)
#define	SC_TORQUE_MODE					(SC_VOLTAGE_CTRL_BIT | SC_CURRENT_CTRL_BIT)
#define	SC_VELOCITY_MODE				(SC_VOLTAGE_CTRL_BIT | SC_CURRENT_CTRL_BIT | SC_VELOCITY_CTRL_BIT)
#define	SC_PROFILE_VELOCITY_MODE		(SC_VOLTAGE_CTRL_BIT | SC_CURRENT_CTRL_BIT | SC_VELOCITY_CTRL_BIT | SC_PROFILE_MODE_BIT)
#define	SC_VELOCITY_AUTO_TUNING_MODE	(SC_VOLTAGE_CTRL_BIT | SC_CURRENT_CTRL_BIT | SC_VELOCITY_CTRL_BIT | SC_VELOCITY_AUTO_TUNING_BIT)
#define	SC_POSITION_MODE				(SC_VOLTAGE_CTRL_BIT | SC_CURRENT_CTRL_BIT | SC_VELOCITY_CTRL_BIT | SC_POSITION_CTRL_BIT)
#define	SC_PROFILE_POSITION_MODE		(SC_VOLTAGE_CTRL_BIT | SC_CURRENT_CTRL_BIT | SC_VELOCITY_CTRL_BIT | SC_POSITION_CTRL_BIT | SC_PROFILE_MODE_BIT)

#define	SC_INIT_TIME_CNT				5000	//pjg++180201


class ServoController : public PositionController
{
public:
	ServoController(CanOpen *canOpenCia402);

protected:
	float Voltage[3];
	float CurrentBuf[3];
	float RBuf[4];
	float LBuf[4];

	uint16_t _ctrlMode;
	
	CanOpen *_canOpenCia402;
	KalmanFilter _kalmanFilter;
	
	uint16_t _nInit;
	#if 1 //12bit
	int16_t _adcResult[4];
	int16_t _adcOffset[2];
	int32_t _adcOffsetSum[2];
	#else //16bit
	uint16_t _adcResult[4];
	uint16_t _adcOffset[2];
	uint32_t _adcOffsetSum[2];
	#endif
	int32_t _adcOffsetOverCnt[2]; //pjg++190509
	
	uint32_t _digitalInput;
	uint32_t _digitalOutput;
	uint32_t _digitalOutputMask;
	uint16_t _analogInput[4];
	
	int16_t _autoTuningIndex;
	int32_t _autoTuningStartPosition;
	
	float _actualTemperature;
	
	MovingAverage _voltageFilter;
	MovingAverage _currentFilter;
	MovingAverage _dAxisCurrentFilter;
	
	//	For parameter estimation
	int32_t run_count;
	int32_t run_count_prev;
	int32_t theta_dt;
	
	void CalculateCurrentOffset(void);
	void CalculateCurrent(void);
	void ControlCurrent(void);
	void ControlVoltage(void);
	
	void InitMachanicParameterEstimator(void);
	void RunMachanicParameterEstimatorOnCurrentController(void);
	void RunMachanicParameterEstimatorOnVelocityController(void);
	
public:
	void LoadProperty(void);
	int8_t Enable(void);
	int8_t Disable(void);
	int8_t SetCtrlMode(uint16_t ctrlMode);
	void SetFault(void);
	void ClearFault(void);
	void SetPositionEncoderSensorValue(int32_t positionEncoderSensorValue);
	
	
	void CalculateVoltage(uint16_t voltage);
	void CalculateTemperature(uint16_t temperature);
	void CalculateElecAngle(uint8_t hallStatus, int32_t encoderPulse);
	void SetElecTheta(float elecTheta);
	void RunCurrentController(int16_t adc0, int16_t adc1);
	void RunVelocityPositionController(void);
	void CalculateLoadTorque(void);
	void SetAnalogInput(uint16_t *analogInput);
	
	void SaveCurrentDataToBuf(void);
	void SaveVelocityDataToBuf(void);
	int CheckUVWire(void);
};
