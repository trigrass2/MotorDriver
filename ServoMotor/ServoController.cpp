#include <stdlib.h>
#include <math.h>
#include "ServoController.h"
#include "Peripheral.h"
#include "utils.h"
#include "Io.h"
#include "Pwm.h"
#include "online_mechanic_param.h"
//#include "stm32f4xx_hal_tim.h" //pjg++181102
#include "stm32f4xx_hal.h" //pjg++181102

ServoController::ServoController(CanOpen *canOpenCia402)
: _kalmanFilter((int32_t)(VELOCITY_CONTROLLER_FREQ), this), _voltageFilter(20, MIN_DC_LINK_VOLTAGE + 5.0f), _currentFilter(CURRENT_MOVING_AVERAGE_FILTER_NUM, 0.0f), _dAxisCurrentFilter(CURRENT_MOVING_AVERAGE_FILTER_NUM, 0.0f)
{
	_ctrlMode = SC_DISABLE_MODE;
	
	_canOpenCia402 = canOpenCia402;
		
	_nInit = 0;

	_adcResult[0] = _adcResult[1] = _adcResult[2] = _adcResult[3] = 0;;
	_adcOffset[0] = _adcOffset[1] = 0;
	_adcOffsetSum[0] = _adcOffsetSum[1] = 0;
	
	_autoTuningIndex = 0;
	_autoTuningStartPosition = 0;
	
	_minActualVoltage = MIN_DC_LINK_VOLTAGE; //pjg<>180202
	//_minActualVoltage = 56;//MIN_DC_LINK_VOLTAGE; //pjg<>180202
	
	_actualTemperature = 12.0f;
}

void ServoController::LoadProperty(void)
{
	//	Motor Properties
	if(_canOpenCia402->_motorType == CIA_402_LINEAR_DC_MOTOR) {
		_motorType = MOTOR_TYPE_LINEAR;
	}
	else {
		_motorType = MOTOR_TYPE_REVOLUTE;
	}
	
	_R = 0.001f * (float)_canOpenCia402->_resistance;						//	10^-3
	_Lq = 0.000001f * (float)_canOpenCia402->_qAxisInductance;				//	10^-6
	_Ld = 0.000001f * (float)_canOpenCia402->_dAxisInductance;				//	10^-6
	_Kt = 0.000001f * (float)_canOpenCia402->_torqueConstant;				//	10^-6
	_Ke = 0.000001f * (float)_canOpenCia402->_backEmfConstant;				//	10^-6
	if(_motorType == MOTOR_TYPE_LINEAR) {
		_Jm = 0.000001f * (float)_canOpenCia402->_systemInertia;			//	10^-6
	}
	else {
		_Jm = 0.0000000001f * (float)_canOpenCia402->_systemInertia;		//	10^-10
	}
	_Fv = 0.000001f * (float)_canOpenCia402->_viscosFriction;				//	10^-6
	_Fc = 0.000001f * (float)_canOpenCia402->_coulombFriction;				//	10^-6
	_elecAngleOffset = (float)_canOpenCia402->_electricAngleOffset*0.1f;	//	10^-1
	_motorPhase = _canOpenCia402->_motorPhase;
	
	_ratedCurrent = (float)_canOpenCia402->_ratedCurrent * 0.001f;
	_maximumCurrent = (float)_canOpenCia402->_maximumCurrent * 0.001f;
	//_maximumCurrent /= 1.5; // pjg++180202 test
	_ratedTorque = (float)_canOpenCia402->_ratedTorque * 0.001f;			//	10^-3
	
	if(_motorType == MOTOR_TYPE_LINEAR) {
		_maximumSpeed = (float)_canOpenCia402->_maxMotorSpeed * 0.001f;
	}
	else {
		_maximumSpeed = (float)_canOpenCia402->_maxMotorSpeed * RPM2RPS;
	}
	
	SetEncoderResolution(_canOpenCia402->_positionEncoderIncrement / _canOpenCia402->_positionEncoderMotorRevolution);
	_polePair = _canOpenCia402->_polePairs;
	
	//	Motion Parameters
	_motionProfileType = _canOpenCia402->_motionProfileType;
	
	if(_motorType == MOTOR_TYPE_LINEAR) {
		_profileVelocity = (float)_canOpenCia402->_profileVelocity * MMPS2MPS;
		_profileAcceleration = (float)_canOpenCia402->_profileAcceleration * MMPS2MPS;
		_profileDeceleration = (float)_canOpenCia402->_profileDeceleration * MMPS2MPS;
		_quickStopDeceleration = (float)_canOpenCia402->_quickStopDeceleration * MMPS2MPS;
		_maximumAcceleration = (float)_canOpenCia402->_maxAcceleration * MMPS2MPS;
		_maximumDeceleration = (float)_canOpenCia402->_maxDeceleration * MMPS2MPS;
	}
	else {
		_profileVelocity = (float)_canOpenCia402->_profileVelocity * RPM2RPS;
		_profileAcceleration = (float)_canOpenCia402->_profileAcceleration * RPM2RPS;
		_profileDeceleration = (float)_canOpenCia402->_profileDeceleration * RPM2RPS;
		_quickStopDeceleration = (float)_canOpenCia402->_quickStopDeceleration * RPM2RPS;
		_maximumAcceleration = (float)_canOpenCia402->_maxAcceleration * RPM2RPS;
		_maximumDeceleration = (float)_canOpenCia402->_maxDeceleration * RPM2RPS;
	}
	
	//	Current Controller
	_qAxisCurrentController.SetPGain((float)_canOpenCia402->_currentPGain * 0.001f);	//	10^-3
	_qAxisCurrentController.SetIGain((float)_canOpenCia402->_currentIGain);				//	10^-0
	
	_dAxisCurrentController.SetPGain((float)_canOpenCia402->_currentPGain * 0.001f);	//	10^-3
	_dAxisCurrentController.SetIGain((float)_canOpenCia402->_currentIGain);				//	10^-0
	
	
	//	Velocity Controller
	if(_motorType == MOTOR_TYPE_LINEAR) {
		_velocityController.SetPGain((float)_canOpenCia402->_velocityPGain * 0.01f);	//	10^-2
		_velocityController.SetIGain((float)_canOpenCia402->_velocityIGain * 0.1f);		//	10^-1
	}
	else {
		_velocityController.SetPGain((float)_canOpenCia402->_velocityPGain * 0.001f);	//	10^-3
		_velocityController.SetIGain((float)_canOpenCia402->_velocityIGain * 0.001f);	//	10^-3
	}
	
	
	//	Position Controller
	
	_positionPGain = (float)_canOpenCia402->_positionPGain * 0.01f;					//	10^-2
}

int8_t ServoController::Enable(void)
{
	EnablePwm();
	
	return 0;
}

int8_t ServoController::Disable(void)
{
	DisablePwm();
	_ctrlMode &= SC_ERROR_BIT;
	
	return 0;
}

int8_t ServoController::SetCtrlMode(uint16_t ctrlMode)
{
	//	에러 상태인 경우
	if((_ctrlMode & SC_ERROR_BIT) == SC_ERROR_BIT)	return -1;
	
	ResetMcData();
	_ctrlMode = ctrlMode;
	
	return 0;
}

void ServoController::SetFault(void)
{
	_ctrlMode |= SC_ERROR_BIT;
}

void ServoController::ClearFault(void)
{
	_ctrlMode &= ~(SC_ERROR_BIT);
	//_hallErrCnt = 0; //pjg++>180830
	//_hallSameCnt = 0; //pjg++180830
	//_encSameCnt = 0; //pjg++180830
	//_canOpenCia402->_hallSameCnt = 0;
	//_canOpenCia402->_encSameCnt = 0; //pjg<++180830
}

void ServoController::SetPositionEncoderSensorValue(int32_t positionEncoderSensorValue)
{
	_actualPosition = positionEncoderSensorValue;
	_prevPosition = positionEncoderSensorValue;
	_demandPosition = positionEncoderSensorValue;
	_targetPosition = positionEncoderSensorValue;
}

void ServoController::CalculateTemperature(uint16_t temperature)
{
	float fTmp = (float)(temperature - 620) * ADC_TO_TEMPERATURE;
		
	_actualTemperature = (TEMPERATURE_LOW_PASS_FILTER_COEF * fTmp + (1.0f - TEMPERATURE_LOW_PASS_FILTER_COEF) * _actualTemperature);
	//_actualTemperature = 30.01; //pjg++190409 test
	if(_nInit < SC_INIT_TIME_CNT) {
		return;
	}
	
	if(_actualTemperature > 70.0f) {
		_canOpenCia402->SetErrorCode(CIA_402_ERROR_CODE_OVER_TEMPERATURE_ERROR);
		_ctrlMode |= SC_ERROR_BIT;
	}
	
	_canOpenCia402->_temperature = (uint8_t)_actualTemperature;

}

//float tempbuf[100];
//uint8_t bufCnt = 0;
void ServoController::CalculateVoltage(uint16_t voltage)
{
	float fTmp = (float)voltage * ADC_TO_VOLTAGE;
	
	//if (fTmp < 40) {
	//	tempbuf[bufCnt++] = fTmp;
	//	if (bufCnt >= 100) bufCnt = 0;
	//}
	
	//	Low-Pass Filter
	_actualVoltage = (VOLTAGE_LOW_PASS_FILTER_COEF * fTmp + (1.0f - VOLTAGE_LOW_PASS_FILTER_COEF) * _actualVoltage);
    //_actualVoltage = 23.37; //pjg++190409 test
	//	Moving Average Filter
	//_actualVoltage = _voltageFilter.Run(fTmp);
	//	No Filter
	//_actualVoltage = fTmp;
	
	if(_nInit < SC_INIT_TIME_CNT) {	//pjg<>180201 occur undervoltage at home-in
		return;
	}
	
	if(_minActualVoltage > _actualVoltage) _minActualVoltage = _actualVoltage;
	if(_maxActualVoltage < _actualVoltage) _maxActualVoltage = _actualVoltage;
		
	_maxOutputVoltage = _actualVoltage * PWM_EFFICIENCY;
	
	if(_actualVoltage > CUT_OFF_DC_LINK_VOLTAGE) {
		TurnOnRegenerativeBrake();
	}
	else {
		TurnOffRegenerativeBrake();
	}
		
	if(_actualVoltage > MAX_DC_LINK_VOLTAGE) {
		_canOpenCia402->SetErrorCode(CIA_402_ERROR_CODE_OVER_VOLTAGE_ERROR);
		_ctrlMode |= SC_ERROR_BIT;
	}
	else if(_actualVoltage < MIN_DC_LINK_VOLTAGE) {
		//_nUnderVoltageCnt++;
		//if (_nUnderVoltageCnt > 3) { // pjg++180202
			_canOpenCia402->SetErrorCode(CIA_402_ERROR_CODE_UNDER_VOLTAGE_ERROR);
			_ctrlMode |= SC_ERROR_BIT;
		//}
	}
	//else _nUnderVoltageCnt = 0;  // pjg++180202
	
	_canOpenCia402->_dcLinkVoltage = (uint32_t)(_actualVoltage * 1000.0f);
}

void ServoController::ControlVoltage(void)
{
	if((_canOpenCia402->_motorType == CIA_402_DC_MOTOR) || (_canOpenCia402->_motorType == CIA_402_LINEAR_DC_MOTOR)) {
		ControlVoltageDC();
	}
	else if(_canOpenCia402->_motorType == CIA_402_BLDC_MOTOR) {
		ControlVoltageBLDC();
	}
	else if(_canOpenCia402->_motorType == CIA_402_PMSM_MOTOR) {
		
		if((_ctrlMode == SC_VOLTAGE_MODE) && (_dAxisVoltage != 0.0f)) {
			_targetElecTheta = 0.0f;
		}
		else {
			_targetElecTheta = _elecTheta + (_elecTheta - _prevElecTheta);
		}
		/*
		if(SearchHallSensorPattern(_hallStatus, _targetElecTheta*RAD_TO_DEG) == 1) {
			
		}
		
		_targetElecTheta += 0.0001f;
		if(_targetElecTheta > 6.283185f) {
			_targetElecTheta = -6.283185f;
		}
		else if(_targetElecTheta < -6.283185f) {
			_targetElecTheta = 6.283185f;
		}
		*/
		
		ControlVoltagePMSM();
	}
	else {
		ResetVoltageController();
	}
}

void ServoController::CalculateCurrentOffset(void)
{
	if(_nInit < SC_INIT_TIME_CNT) { //pjg++190326
		if (_adcResult[0] < 10 || _adcResult[1] < 10) return;
	}
	_adcOffsetSum[0] += _adcResult[0];
	_adcOffsetSum[1] += _adcResult[1];
		
	if(++_nInit == SC_INIT_TIME_CNT) {
		_adcOffset[0] = (int16_t)(_adcOffsetSum[0] / SC_INIT_TIME_CNT);
		_adcOffset[1] = (int16_t)(_adcOffsetSum[1] / SC_INIT_TIME_CNT);
		
		_canOpenCia402->_statusWord = CIA_402_STATUS_SWITCH_ON_DISABLED | CIA_402_REMOTE;
	}
}

void ServoController::CalculateCurrent(void)
{
	if((_adcOffset[0] > 2304) || (_adcOffset[0] < 1792) || (_adcOffset[1] > 2304) || (_adcOffset[1] < 1792)) {
		_canOpenCia402->SetErrorCode(CIA_402_ERROR_CODE_CURRENT_DETECTION_ERROR);
		_ctrlMode |= SC_ERROR_BIT;
	}
	
	//	Calculte the actual current
	switch(_motorPhase) {
		default :
		case MOTOR_PHASE_U_V_W :
			_uCurrent = (float)(_adcResult[0] - _adcOffset[0]) * ADC_TO_CURRENT;
			_vCurrent = (float)(_adcResult[1] - _adcOffset[1]) * ADC_TO_CURRENT;
			_wCurrent = -(_uCurrent + _vCurrent);
			break;

		case MOTOR_PHASE_U_W_V :
			_uCurrent = (float)(_adcResult[0] - _adcOffset[0]) * ADC_TO_CURRENT;
			_wCurrent = (float)(_adcResult[1] - _adcOffset[1]) * ADC_TO_CURRENT;
			_vCurrent = -(_uCurrent + _wCurrent);
			break;
			
		case MOTOR_PHASE_V_U_W :
			_vCurrent = (float)(_adcResult[0] - _adcOffset[0]) * ADC_TO_CURRENT;
			_uCurrent = (float)(_adcResult[1] - _adcOffset[1]) * ADC_TO_CURRENT;
			_wCurrent = -(_vCurrent + _uCurrent);
			break;
			
		case MOTOR_PHASE_V_W_U :
			_vCurrent = (float)(_adcResult[0] - _adcOffset[0]) * ADC_TO_CURRENT;
			_wCurrent = (float)(_adcResult[1] - _adcOffset[1]) * ADC_TO_CURRENT;
			_uCurrent = -(_vCurrent + _wCurrent);
			break;
			
		case MOTOR_PHASE_W_U_V :
			_wCurrent = (float)(_adcResult[0] - _adcOffset[0]) * ADC_TO_CURRENT;
			_uCurrent = (float)(_adcResult[1] - _adcOffset[1]) * ADC_TO_CURRENT;
			_vCurrent = -(_wCurrent + _uCurrent);
			break;
			
		case MOTOR_PHASE_W_V_U :
			_wCurrent = (float)(_adcResult[0] - _adcOffset[0]) * ADC_TO_CURRENT;
			_vCurrent = (float)(_adcResult[1] - _adcOffset[1]) * ADC_TO_CURRENT;
			_uCurrent = -(_wCurrent + _vCurrent);
			break;
	}

	
	if((_canOpenCia402->_motorType == CIA_402_DC_MOTOR) || (_canOpenCia402->_motorType == CIA_402_LINEAR_DC_MOTOR)) {
		CalculateCurrentDC();
	}
	else if(_canOpenCia402->_motorType == CIA_402_BLDC_MOTOR) {
		CalculateCurrentBLDC();
	}
	else if(_canOpenCia402->_motorType == CIA_402_PMSM_MOTOR) {
		CalculateCurrentPMSM();
	}
	
	//	Calculate the averaged current
	//	Low-Pass Filter
	//_actualCurrent = (CURRENT_LOW_PASS_FILTER_COEF * _qAxisActualCurrent + (1.0f - CURRENT_LOW_PASS_FILTER_COEF) * _actualCurrent);
	//	Moving Average Filter
	_actualCurrent = _qAxisAveragedCurrent = _currentFilter.Run(_qAxisActualCurrent);
	_dAxisAveragedCurrent = _dAxisCurrentFilter.Run(_dAxisActualCurrent);
	
	//	Update variables for CiA-402 Device Profile
	_canOpenCia402->_dAxisVoltage = (int32_t)(_dAxisVoltage * 1000.0f);
	_canOpenCia402->_qAxisVoltage = (int32_t)(_qAxisVoltage * 1000.0f);
	
	_canOpenCia402->_dAxisTargetCurrent = (int32_t)(_dAxisTargetCurrent * 1000.0f);
	_canOpenCia402->_qAxisTargetCurrent = (int32_t)(_qAxisTargetCurrent * 1000.0f);
	_canOpenCia402->_dAxisActualCurrent = (int32_t)(_dAxisActualCurrent * 1000.0f);
	_canOpenCia402->_qAxisActualCurrent = (int16_t)(_qAxisActualCurrent * 1000.0f); //pjg--180207
	//_canOpenCia402->_qAxisActualCurrent = (int32_t)(_qAxisActualCurrent * 1000.0f); //pjg<>180207 type change
	
	_canOpenCia402->_actualCurrent = (int16_t)(_qAxisActualCurrent * 1000.0f);
	_canOpenCia402->_averagedCurrent = (int32_t)(_actualCurrent * 1000.0f);
	
	_canOpenCia402->_actualTorque = (int16_t)(_actualCurrent/_ratedCurrent*1000.0f);
}

void ServoController::ControlCurrent(void)
{
	//_currentOffset = (float)_canOpenCia402->_torqueOffset * 0.001f * _ratedCurrent ; //pjg--181130
	_currentOffset = (float)_canOpenCia402->_torqueOffset * 0.001f * _ratedCurrent + _currentOffsetable; //pjg<>181130
	
	if((_canOpenCia402->_motorType == CIA_402_DC_MOTOR) || (_canOpenCia402->_motorType == CIA_402_LINEAR_DC_MOTOR)) {
		RunCurrentControllerDC();
	}
	else if(_canOpenCia402->_motorType == CIA_402_BLDC_MOTOR) {
		RunCurrentControllerBLDC();
	}
	else if(_canOpenCia402->_motorType == CIA_402_PMSM_MOTOR) {
		RunCurrentControllerPMSM();
	}
	else {
		_dAxisVoltage = 0.0f;
		_qAxisVoltage = 0.0f;
	}
}

void ServoController::CalculateElecAngle(uint8_t hallStatus, int32_t encoderPulse)
{
	int ret;
	
	if(_canOpenCia402->_positionSensorPolarity & 0x02) {
		_hallStatus = (~hallStatus)&0x07;
	}
	else {
		_hallStatus = hallStatus;
	}
	
	_prevElecTheta = _elecTheta;
	
	if(_canOpenCia402->_motorType != CIA_402_PMSM_MOTOR) {
		return;
	}
	
	if(_ctrlMode == SC_CURRENT_AUTO_TUNING_MODE) {
		_elecTheta = 0.0f;
		_elecSinTheta = arm_sin_f32(0.0f);
		_elecCosTheta = arm_cos_f32(0.0f);
	}
	else {
		if(_canOpenCia402->_positionSensorPolarity & 0x01) {
			//if(CalculateElecTheta(-encoderPulse) < 0) {   //sec<>171204
			ret = CalculateElecTheta(-encoderPulse); //pjg<>180831
		}
		else {
			//if(CalculateElecTheta(encoderPulse) < 0) {   //sec<>171204
			ret = CalculateElecTheta(encoderPulse); //pjg<>180831
		}
		if(ret < 0) { //pjg<>180831
			if (ret == -1) 	_canOpenCia402->SetErrorCode(CIA_402_ERROR_CODE_HALL_SENSOR_NOT_FOUND_ERROR);
			else if (ret == -2) _canOpenCia402->SetErrorCode(CIA_402_ERROR_CODE_HALL_SENSOR_ERROR);
			else if (ret == -3) _canOpenCia402->SetErrorCode(CIA_402_ERROR_CODE_HALL_ANGLE_DETECTION_ERROR);
			else if (ret == -4) _canOpenCia402->SetErrorCode(CIA_402_ERROR_CODE_POSITION_SENSOR_BREACH_ERROR);
			else _canOpenCia402->SetErrorCode(CIA_402_ERROR_CODE_ENCODER_DISCONNECTION_ERROR);
		}
		//_canOpenCia402->_hallSameCnt = _hallSameCnt;  //pjg++>180827
		//_canOpenCia402->_encSameCnt = _encSameCnt; //pjg++>180827
	}
}

void ServoController::SetElecTheta(float elecTheta)
{
	/*
	float tmp = (elecTheta - _prevElecTheta);
	_prevElecTheta = elecTheta;
		
	_elecTheta = (elecTheta + tmp) * (float)_polePair + (_elecAngleOffset * DEG_TO_RAD);
	_elecSinTheta = arm_sin_f32(_elecTheta);
	_elecCosTheta = arm_cos_f32(_elecTheta);
	*/
}

void ServoController::RunCurrentController(int16_t adc0, int16_t adc1)
{
	_adcResult[0] = adc0;
	_adcResult[1] = adc1;
	
	if(_nInit < SC_INIT_TIME_CNT) {
		DisablePwm();
		CalculateCurrentOffset();
		
		if(_nInit == SC_INIT_TIME_CNT) {
			TurnOffErrorLed();
		}
		return;
	}
	
	CalculateCurrent();
	
	if(CheckOverCurrent() < 0) {
		_canOpenCia402->SetErrorCode(CIA_402_ERROR_CODE_OVER_CURRENT_ERROR);
		_ctrlMode |= SC_ERROR_BIT;
	}

	//	Check Error
	if((_ctrlMode & SC_ERROR_BIT) == SC_ERROR_BIT) {
		ResetVoltageController();
		ResetCurrentController();
		DisablePwm();
		_pwm1 = _pwm2 = _pwm3 = -1;
	}

	if(_ctrlMode == SC_CURRENT_AUTO_TUNING_MODE) {
		if((_canOpenCia402->_controlWord & CIA_402_CONTROL_START_AUTO_TUNING) == CIA_402_CONTROL_START_AUTO_TUNING) {
			_canOpenCia402->_controlWord &= ~CIA_402_CONTROL_START_AUTO_TUNING;

			_autoTuningIndex = 0;
			_canOpenCia402->_bufIndex = 0;
			
			Voltage[0] = -1.0f;
			Voltage[1] = -2.0f;
			Voltage[2] = -3.0f;
			
			CurrentBuf[0] = CurrentBuf[1] = CurrentBuf[2] = 0.0f;
		}
		
		///////////////////////////////////////////////////////////////////////
		//
		//	Current Auto-Tunning - Start
		//
		///////////////////////////////////////////////////////////////////////
		if(!(_canOpenCia402->_statusWord & CIA_402_SUCCESS_AUTO_TUNING)) {
			if(_autoTuningIndex == 0) {
				_dAxisVoltage = Voltage[0];
				_qAxisVoltage = 0.0f;
			}
			else if(_autoTuningIndex == 250) {
				_dAxisVoltage = Voltage[1];
				_qAxisVoltage = 0.0f;
			}
			else if(_autoTuningIndex == 500) {
				_dAxisVoltage = Voltage[2];
				_qAxisVoltage = 0.0f;
			}
			else if(_autoTuningIndex == 750) {
				_dAxisVoltage = 0.0f;
				_qAxisVoltage = 0.0f;
			}
			
			if(_autoTuningIndex >= 200 && _autoTuningIndex < 250) {
				CurrentBuf[0] += _dAxisActualCurrent;
			}
			else if(_autoTuningIndex >= 450 && _autoTuningIndex < 500) {
				CurrentBuf[1] += _dAxisActualCurrent;
			}
			else if(_autoTuningIndex >= 700 && _autoTuningIndex < 750) {
				CurrentBuf[2] += _dAxisActualCurrent;
			}
			
			if(++_autoTuningIndex == 1000) {
				_canOpenCia402->_statusWord |= CIA_402_SUCCESS_AUTO_TUNING;
				//	Calculate Resistance
				CurrentBuf[0] /= 50.0f;
				CurrentBuf[1] /= 50.0f;
				CurrentBuf[2] /= 50.0f;
				
				RBuf[0] = Voltage[0] / CurrentBuf[0];
				RBuf[1] = Voltage[1] / CurrentBuf[1];
				RBuf[2] = Voltage[2] / CurrentBuf[2];
				//RBuf[3] = -(RBuf[0] + RBuf[1] + RBuf[2]) / 3.0f;
				RBuf[3] = (RBuf[1] + RBuf[2]) * 0.5f;
				
				_R = RBuf[3]*0.5f;
				
				//	Calculate Inductance
				CurrentBuf[0] *= -1.0f;
				CurrentBuf[1] *= -1.0f;
				CurrentBuf[2] *= -1.0f;
				
				for(_autoTuningIndex = 0; _autoTuningIndex < 245; _autoTuningIndex++) {
					LBuf[0] = (_canOpenCia402->_buf1[_autoTuningIndex] + _canOpenCia402->_buf1[_autoTuningIndex+1] + _canOpenCia402->_buf1[_autoTuningIndex+2] + _canOpenCia402->_buf1[_autoTuningIndex+3]) * -0.00025f;
					if(LBuf[0] > CurrentBuf[0]*0.632f) {
						LBuf[0] = 2.0f/3.0f*_R*(float)_autoTuningIndex*CURRENT_CONTROLLER_PERIOD;
						break;
					}
				}
				for(_autoTuningIndex = 250; _autoTuningIndex < 495; _autoTuningIndex++) {
					LBuf[1] = (_canOpenCia402->_buf1[_autoTuningIndex] + _canOpenCia402->_buf1[_autoTuningIndex+1] + _canOpenCia402->_buf1[_autoTuningIndex+2] + _canOpenCia402->_buf1[_autoTuningIndex+3]) * -0.00025f;
					if(LBuf[1] > (CurrentBuf[1]-CurrentBuf[0])*0.623f+CurrentBuf[0]) {
						LBuf[1] = 2.0f/3.0f*_R*(float)(_autoTuningIndex-250)*CURRENT_CONTROLLER_PERIOD;
						break;
					}
				}
				for(_autoTuningIndex = 500; _autoTuningIndex < 745; _autoTuningIndex++) {
					LBuf[2] = (_canOpenCia402->_buf1[_autoTuningIndex] + _canOpenCia402->_buf1[_autoTuningIndex+1] + _canOpenCia402->_buf1[_autoTuningIndex+2] + _canOpenCia402->_buf1[_autoTuningIndex+3]) * -0.00025f;
					if(LBuf[2] > (CurrentBuf[2]-CurrentBuf[1])*0.623f+CurrentBuf[1]) {
						LBuf[2] = 2.0f/3.0f*_R*(float)(_autoTuningIndex-500)*CURRENT_CONTROLLER_PERIOD;
						break;
					}
				}
				
				//LBuf[3] = (LBuf[0]+LBuf[1]+LBuf[2])/3.0f;
				LBuf[3] = (LBuf[1]+LBuf[2])*0.5f;
			}
			
			ControlVoltage();
		}
		///////////////////////////////////////////////////////////////////////
		//
		//	Current Auto-Tunning - End
		//
		///////////////////////////////////////////////////////////////////////
	}
	else {
		//	Current Control
		if((_ctrlMode & SC_CURRENT_CTRL_BIT) == SC_CURRENT_CTRL_BIT)	ControlCurrent();
		else															ResetCurrentController();
		
		//	Voltage Control
		if((_ctrlMode & SC_VOLTAGE_CTRL_BIT) == SC_VOLTAGE_CTRL_BIT)	ControlVoltage();
		else															ResetCurrentController();
		//else															ResetVoltageController(); //pjg<>171128
	}

	//	Servo Off	
	if((_ctrlMode & (~SC_ERROR_BIT)) == SC_DISABLE_MODE) {
		ResetVoltageController();
		ResetCurrentController();
	}
	
	SetPwm(_pwm1, _pwm2, _pwm3);
	
	_prevHallStatus = _hallStatus;
	_canOpenCia402->_hallSensorPattern = _hallStatus;
		
	SaveCurrentDataToBuf();
}


void ServoController::RunVelocityPositionController(void)
{
	static uint16_t prevMotionType;
	float J, B, C;
	//	Get the Digital Input States
	if(_nInit >= SC_INIT_TIME_CNT) {
		_digitalInput = GetDigitalInput();
		_digitalInput ^= _canOpenCia402->_digitalInputPolarity; //pjg++190503
		_digitalInput &= _canOpenCia402->_digitalInputMask; //pjg++190503
	}
	
	//	Velocity, Position and Homing Control
	//	Check Error
	if((_ctrlMode & SC_ERROR_BIT) == SC_ERROR_BIT) {
		ResetVelocityController();
		ResetPositionController();
	}
	else if(_ctrlMode == SC_VELOCITY_MODE) {
		RunVelocityController();
	}
	else if(_ctrlMode == SC_PROFILE_VELOCITY_MODE) {
		RunProfileVelocityController();
	}
	else if(_ctrlMode == SC_POSITION_MODE) {
		RunPositionController();
		RunVelocityController();
	}
	else if(_ctrlMode == SC_PROFILE_POSITION_MODE) {
		RunProfilePositionController();
		RunVelocityController();
	}
	else if(_ctrlMode == SC_VELOCITY_AUTO_TUNING_MODE) {
		if((_canOpenCia402->_controlWord & CIA_402_CONTROL_START_AUTO_TUNING) == CIA_402_CONTROL_START_AUTO_TUNING) {
			_canOpenCia402->_controlWord &= ~CIA_402_CONTROL_START_AUTO_TUNING;
			ResetMcData();
			_autoTuningStartPosition = _actualPosition;
		
			_profileVelocity = _maximumSpeed * 0.5f;
			_profileAcceleration = _maximumAcceleration * 0.5f;
			_profileDeceleration = _maximumDeceleration * 0.5f;
			
			_Jm = 0.0f;
			_Fv = 0.0f;
			_Fc = 0.0f;
			_autoTuningIndex = 0;
			prevMotionType = _motionProfileType;
			_motionProfileType = 0;
		}
		
		///////////////////////////////////////////////////////////////////////
		//
		//	Velocity Auto-Tunning - Start
		//
		///////////////////////////////////////////////////////////////////////
		if(!(_canOpenCia402->_statusWord & CIA_402_SUCCESS_AUTO_TUNING)) {
			if((_autoTuningIndex % 4000) == 0) {
				SetTargetProfilePosition(_autoTuningStartPosition + _canOpenCia402->_velocityAutoTuningPosition);
			}
			else if((_autoTuningIndex % 4000) == 1000) {
				SetTargetProfilePosition(_autoTuningStartPosition);
			}
			else if((_autoTuningIndex % 4000) == 2000) {
				SetTargetProfilePosition(_autoTuningStartPosition - _canOpenCia402->_velocityAutoTuningPosition);
			}
			else if((_autoTuningIndex % 4000) == 3000) {
				SetTargetProfilePosition(_autoTuningStartPosition);
			}
			
			if(_motorType == MOTOR_TYPE_LINEAR) {
				AddMcData(_actualPosition, _actualVelocity, _actualAcceleration, _actualCurrent*_Kt, _pulse2rad, 0.001f);	//	Threshold : 1mm/sec
			}
			else {
				AddMcData(_actualPosition, _actualVelocity, _actualAcceleration, _actualCurrent*_Kt, _pulse2rad, M_PI*0.1f);		//	Threshold : 1rad/sec	-> 314RPM
			}
			
			if(++_autoTuningIndex >= 4000) {
				if(CalcMcParam (J, B, C) == true) {
					if(J == 0) return;
					_Jm = J;
					_Fv = B;
					_Fc = C;
					
					_canOpenCia402->_statusWord |= CIA_402_SUCCESS_AUTO_TUNING;
					
					_kalmanFilter.Init();
					
					if(_motorType == MOTOR_TYPE_LINEAR) {
						_canOpenCia402->_systemInertia = (uint32_t)(_Jm * 1000000.0f);	//	g -> ug -> 10^6
					}
					else {
						_canOpenCia402->_systemInertia = (uint32_t)(_Jm * 10000000000.0f);	//	kg*m^2 -> ug*cm^2 -> 10^10
					}
					
					_velocityController.SetPGain(_Jm*(float)_canOpenCia402->_velocityControllerBandwidth/_Kt);
					_velocityController.SetIGain(_Jm*(float)_canOpenCia402->_velocityControllerBandwidth/_Kt*(float)_canOpenCia402->_velocityControllerBandwidth/5.0f);
					
					_canOpenCia402->_coulombFriction = (uint32_t)(_Fc * 1000000.0f);
					_canOpenCia402->_viscosFriction = (uint32_t)(_Fv * 1000000.0f);
									
					if(_motorType == MOTOR_TYPE_LINEAR) {
						_profileVelocity = (float)_canOpenCia402->_profileVelocity * MMPS2MPS;
						_canOpenCia402->_velocityPGain = (uint16_t)(_Jm*(float)_canOpenCia402->_velocityControllerBandwidth/_Kt * 100.0f);
						_canOpenCia402->_velocityIGain = (uint16_t)(_Jm*(float)_canOpenCia402->_velocityControllerBandwidth/_Kt * (float)_canOpenCia402->_velocityControllerBandwidth/5.0f*10.0f);
					}
					else {
						_profileVelocity = (float)_canOpenCia402->_profileVelocity * RPM2RPS;
						_canOpenCia402->_velocityPGain = (uint16_t)(_Jm*(float)_canOpenCia402->_velocityControllerBandwidth/_Kt * 1000.0f);
						_canOpenCia402->_velocityIGain = (uint16_t)(_Jm*(float)_canOpenCia402->_velocityControllerBandwidth/_Kt * (float)_canOpenCia402->_velocityControllerBandwidth/5.0f*1000.0f);
					}
					_motionProfileType = prevMotionType;
					SetTargetProfilePosition(_autoTuningStartPosition);
				}
				_autoTuningIndex = 0;	
			}
		}
		///////////////////////////////////////////////////////////////////////
		//
		//	Velocity Auto-Tunning - End
		//
		///////////////////////////////////////////////////////////////////////
		
		RunProfilePositionController();
		RunVelocityController();
	}
	else {
		ResetVelocityController();
		ResetPositionController();
	}

	/*
	///////////////////////////////////////////////////////////////////////
	//
	//	Calculate the error of electric theta
	//
	///////////////////////////////////////////////////////////////////////
	float x, y, tmp;
	
	x = (_qAxisVoltage - _R*_qAxisActualCurrent - _Lq*(_qAxisActualCurrent - _qAxisPrevActualCurrent)*VELOCITY_CONTROLLER_FREQ - _Ld*_dAxisActualCurrent*_actualVelocity);
	if(_actualVelocity < 0.0f) 	x = -x;
	
	y = (_dAxisVoltage - _R*_dAxisActualCurrent - _Ld*(_dAxisActualCurrent - _dAxisPrevActualCurrent)*VELOCITY_CONTROLLER_FREQ + _Lq*_qAxisActualCurrent*_actualVelocity);
	if(_actualVelocity < 0.0f) 	y = -y;
	
	tmp = atan2(y, x) * RAD_TO_DEG;
	
	if(fabs(_actualVelocity) > M_PI*10.0f) {
		_elecThetaError = (ELEC_THETA_LOW_PASS_FILTER_COEF * tmp + (1.0f - ELEC_THETA_LOW_PASS_FILTER_COEF) * _prevElecThetaError);
		_prevElecThetaError = _elecThetaError;
	}
	else if(_actualVelocity > 0.0f) {
		_prevElecThetaError = _elecThetaError = -0.5f;
	}
	else if(_actualVelocity < 0.0f) {
		_prevElecThetaError = _elecThetaError = 0.5f;
	}
	else {
		_prevElecThetaError = _elecThetaError = 0.0f;
	}
	
	_dAxisPrevActualCurrent = _dAxisActualCurrent;
	_qAxisPrevActualCurrent = _qAxisActualCurrent;
	*/
	
	
	//	Update variables for CiA-402 Profile
	if(_motorType == MOTOR_TYPE_LINEAR) {
		_canOpenCia402->_actualVelocity = (int32_t)(_actualVelocity * MPS2MMPS);	//	m/sec -> mm/sec
		_canOpenCia402->_demandVelocity = (int32_t)(_demandVelocity * MPS2MMPS);	//	m/sec -> mm/sec
	}
	else {
		_canOpenCia402->_actualVelocity = (int32_t)(_actualVelocity * RPS2RPM);		//	rad/sec -> RPM
		_canOpenCia402->_demandVelocity = (int32_t)(_demandVelocity * RPS2RPM);		//	rad/sec -> RPM
	}
	
	_canOpenCia402->_demandPosition = _demandPosition;
	_canOpenCia402->_actualPosition = _actualPosition;
	
	_canOpenCia402->_digitalInput = _digitalInput;
	
	_canOpenCia402->UpdateStatusWord();
	//if (_canOpenCia402->_fHallSameCntClear) _hallSameCnt = 0; //pjg++180828
	//if (_canOpenCia402->_fEncSameCntClear) _encSameCnt = 0; //pjg++180828
	if (_canOpenCia402->_statusWord & CIA_402_FAULT) {
		_ctrlMode |= SC_ERROR_BIT; //pjg++180827
	}
	
	//	Update LED Status
	if((_ctrlMode & SC_ERROR_BIT) == SC_ERROR_BIT)	TurnOnErrorLed();
	else											TurnOffErrorLed();
	
	if(_actualVelocity > 0.0f) {
		TurnOnStatusLed(0);
		TurnOffStatusLed(1);
	}
	else if(_actualVelocity < -0.0f) {
		TurnOffStatusLed(0);
		TurnOnStatusLed(1);
	}
	else {
		TurnOffStatusLedAll();
	}
	
	SaveVelocityDataToBuf();
}

void ServoController::CalculateLoadTorque(void)
{
	if((_ctrlMode & SC_VELOCITY_AUTO_TUNING_MODE) == SC_VELOCITY_AUTO_TUNING_MODE) {
		_kalmanFilter.Init();
	}
	else if((_ctrlMode & SC_VELOCITY_MODE) == SC_VELOCITY_MODE) {
		_kalmanFilter.Predict(_actualCurrent - _currentOffset);
		_kalmanFilter.Update(_actualVelocity);
	}
	else {
		_kalmanFilter.Init();
	}
	
	_estimatedVelocity = _kalmanFilter.v_hat;
	_Tl = _kalmanFilter.Tl_hat;
	_canOpenCia402->_loadTorque = (int32_t)(_Tl * 1000.0f);
}

void ServoController::SetAnalogInput(uint16_t *analogInput)
{
	for(int i = 0; i < 4; i++) {
		_canOpenCia402->_analogInput[i] = _analogInput[i] = *(analogInput + 0);
	}
}

void ServoController::SaveCurrentDataToBuf(void)
{
	if(_ctrlMode == SC_CURRENT_AUTO_TUNING_MODE) {
		if(_canOpenCia402->_bufIndex >= 0){
			_canOpenCia402->_buf0[_canOpenCia402->_bufIndex] = (int32_t)(_dAxisVoltage * 1000.0f);
			_canOpenCia402->_buf1[_canOpenCia402->_bufIndex] = (int32_t)(_dAxisActualCurrent * 1000.0f);
			_canOpenCia402->_bufIndex++;
		}
		
		if(_canOpenCia402->_bufIndex >= 1000) {
			_canOpenCia402->_bufIndex = -1;
		}
	}
	else {
		if(_canOpenCia402->_bufIndex >= 0){
			if(	(_canOpenCia402->_modesOfOperationDisplay == CIA_402_CYCLIC_SYNC_TORQUE_MODE) ||
				(_canOpenCia402->_modesOfOperationDisplay == CIA_402_CURRENT_MODE)||
				(_canOpenCia402->_modesOfOperationDisplay == CIA_402_TORQUE_MODE)) {

				_canOpenCia402->_buf0[_canOpenCia402->_bufIndex] = (int32_t)(_qAxisVoltage * 1000.0f);
				_canOpenCia402->_buf1[_canOpenCia402->_bufIndex] = (int32_t)(_qAxisActualCurrent * 1000.0f);
				_canOpenCia402->_bufIndex++;
			}
		}
		
		if(_canOpenCia402->_bufIndex >= 1000) {
			_canOpenCia402->_bufIndex = -1;
		}
	}
}

void ServoController::SaveVelocityDataToBuf(void)
{
	static int nSaveVelocityDataToBuf = 0;
	
	if(++nSaveVelocityDataToBuf < 4) {
		return;
	}
	else {
		nSaveVelocityDataToBuf = 0;
	}
	
	if(_canOpenCia402->_bufIndex >= 0){
		if(	(_canOpenCia402->_modesOfOperationDisplay == CIA_402_CYCLIC_SYNC_VELOCITY_MODE) ||
			(_canOpenCia402->_modesOfOperationDisplay == CIA_402_PROFILE_VELOCITY_MODE) ||
			(_canOpenCia402->_modesOfOperationDisplay == CIA_402_VOLTAGE_MODE)||
			(_canOpenCia402->_modesOfOperationDisplay == CIA_402_POSITION_CURRENT_POSITION_MODE)||
	   		(_canOpenCia402->_modesOfOperationDisplay == CIA_402_VELOCITY_MODE)) {

			_canOpenCia402->_buf0[_canOpenCia402->_bufIndex] = (int32_t)_canOpenCia402->_actualCurrent;
			_canOpenCia402->_buf1[_canOpenCia402->_bufIndex] = _canOpenCia402->_actualVelocity;
			_canOpenCia402->_bufIndex++;
		}
		else if((_canOpenCia402->_modesOfOperationDisplay == CIA_402_CYCLIC_SYNC_POSITION_MODE) ||
			(_canOpenCia402->_modesOfOperationDisplay == CIA_402_PROFILE_POSITION_MODE) ||
			(_canOpenCia402->_modesOfOperationDisplay == CIA_402_POSITION_MODE)) {
			_canOpenCia402->_buf0[_canOpenCia402->_bufIndex] = _canOpenCia402->_actualVelocity;
			_canOpenCia402->_buf1[_canOpenCia402->_bufIndex] = _canOpenCia402->_actualPosition;
			_canOpenCia402->_bufIndex++;
		}
	}
	
	if(_canOpenCia402->_bufIndex >= 1000) {
		_canOpenCia402->_bufIndex = -1;
	}
}

