#include <arm_math.h>
#include "peripheral.h"
#include "CurrentController.h"
#include "FieldOrientedControl.h"

#define	BLDC_PHASE_U_TO_V	0
#define	BLDC_PHASE_U_TO_W	1
#define	BLDC_PHASE_V_TO_W	2
#define	BLDC_PHASE_V_TO_U	3
#define	BLDC_PHASE_W_TO_U	4
#define	BLDC_PHASE_W_TO_V	5

CurrentController::CurrentController(void)
: _dAxisCurrentController(CURRENT_CONTROLLER_PERIOD), _qAxisCurrentController(CURRENT_CONTROLLER_PERIOD)
{
	//	Voltage
	_actualVoltage = 24.0f;
	_maxOutputVoltage = _actualVoltage * 0.95f;
	
	//	Current
	_uCurrent = 0.0f;
	_vCurrent = 0.0f;
	_wCurrent = 0.0f;
	
	_nUOverCurrent = 0;
	_nVOverCurrent = 0;
	_nWOverCurrent = 0;
	_nOverCurrent = 0;
	
	_actualCurrent = 0.0f;
	_dAxisActualCurrent = 0.0f;
	_qAxisActualCurrent = 0.0f;
	
	_dAxisPrevActualCurrent = 0.0f;
	_qAxisPrevActualCurrent = 0.0f;
	
	_dAxisAveragedCurrent = 0.0f;
	_qAxisAveragedCurrent = 0.0f;
	
	_currentOffset = 0.0f;
		
	//	Hall Sensor
	_hallStatus = 0;
	_prevHallStatus = 0;
	_hallSector = 0;
	
	//	Electric Angle(theta)
	_focStatus = 0;
	_elecActualPosition = 0;
	_elecPrevPosition = 0;
	
	_elecInitAngle = 0.0f;
	_elecAngleOffset = 0.0f;
	
	_elecTheta = 0.0f;
	_prevElecTheta = 0.0f;
	_elecSinTheta = 0.0f;
	_elecCosTheta = 0.0f;
	_elecThetaError = 0.0f;
	_prevElecThetaError = 0.0f;
	
	//	Controller
	_backEmfVelocity = 0.0f;
	
	ResetVoltageController();
	ResetCurrentController();
}

void CurrentController::ResetVoltageController(void)
{
	//	PWM
	_pwm1 = 0;
	_pwm2 = 0;
	_pwm3 = 0;
	
	//	Voltage
	_uVoltage = 0.0f;
	_vVoltage = 0.0f;
	_wVoltage = 0.0f;
		
	_dAxisVoltage = 0.0f;
	_qAxisVoltage = 0.0f;
}

void CurrentController::SetDAxisVoltage(float dAxisVoltage)
{
	if(dAxisVoltage > _actualVoltage)		_dAxisVoltage = _actualVoltage;
	else 									_dAxisVoltage = dAxisVoltage;
}

void CurrentController::SetQAxisVoltage(float qAxisVoltage)
{
	if(qAxisVoltage > _actualVoltage)		_qAxisVoltage = _actualVoltage;
	else									_qAxisVoltage = qAxisVoltage;
}

void CurrentController::ControlVoltageDC(void)
{
	int16_t targetPwm = (int16_t)((float)MAX_PWM * _qAxisVoltage / _maxOutputVoltage);
	
	if(targetPwm == 0) {
		_pwm1 = MAX_PWM_INV_2;
		_pwm2 = MAX_PWM_INV_2;
	}
	else {
		_pwm1 = MAX_PWM_INV_2 + targetPwm / 2;
		_pwm2 = MAX_PWM_INV_2 - targetPwm / 2;
	}
	
	_pwm3 = -1;
}

void CurrentController::ControlVoltageBLDC(void)
{
	int16_t pwm1, pwm2, pwm3;
	float targetPwm = (float)MAX_PWM * _qAxisVoltage / _maxOutputVoltage;
	
	
	
	/*
	pwm1 = (int16_t)(targetPwm * BLDC_PHASE_U[_hallSector]);
	pwm2 = (int16_t)(targetPwm * BLDC_PHASE_V[_hallSector]);
	pwm3 = (int16_t)(targetPwm * BLDC_PHASE_W[_hallSector]);
	*/
	
	switch(_motorPhase) {
		default :
		case MOTOR_PHASE_U_V_W :
			_pwm1 = pwm1;	_pwm2 = pwm2;	_pwm3 = pwm3;
			break;

		case MOTOR_PHASE_U_W_V :
			_pwm1 = pwm1;	_pwm2 = pwm3;	_pwm3 = pwm2;
			break;
			
		case MOTOR_PHASE_V_U_W :
			_pwm1 = pwm2;	_pwm2 = pwm1;	_pwm3 = pwm3;
			break;
			
		case MOTOR_PHASE_V_W_U :
			_pwm1 = pwm2;	_pwm2 = pwm3;	_pwm3 = pwm1;
			break;
			
		case MOTOR_PHASE_W_U_V :
			_pwm1 = pwm3;	_pwm2 = pwm1;	_pwm3 = pwm2;
			break;
			
		case MOTOR_PHASE_W_V_U :
			_pwm1 = pwm3;	_pwm2 = pwm2;	_pwm3 = pwm1;
			break;
	}
}

void CurrentController::ControlVoltagePMSM(void)
{
	int16_t pwm1, pwm2, pwm3;
	
	_elecSinTheta = arm_sin_f32(_targetElecTheta);
	_elecCosTheta = arm_cos_f32(_targetElecTheta);
	
	InverseClarkeParkTransform(_elecSinTheta, _elecCosTheta, _dAxisVoltage, _qAxisVoltage, &_uVoltage, &_vVoltage, &_wVoltage);
	
	MakeSpaceVectorPwm(_actualVoltage, _maxOutputVoltage, _uVoltage, _vVoltage, _wVoltage, MAX_PWM, &pwm1, &pwm2, &pwm3);
	
	switch(_motorPhase) {
		default :
		case MOTOR_PHASE_U_V_W :
			_pwm1 = pwm1;	_pwm2 = pwm2;	_pwm3 = pwm3;
			break;

		case MOTOR_PHASE_U_W_V :
			_pwm1 = pwm1;	_pwm2 = pwm3;	_pwm3 = pwm2;
			break;
			
		case MOTOR_PHASE_V_U_W :
			_pwm1 = pwm2;	_pwm2 = pwm1;	_pwm3 = pwm3;
			break;
			
		case MOTOR_PHASE_V_W_U :
			_pwm1 = pwm2;	_pwm2 = pwm3;	_pwm3 = pwm1;
			break;
			
		case MOTOR_PHASE_W_U_V :
			_pwm1 = pwm3;	_pwm2 = pwm1;	_pwm3 = pwm2;
			break;
			
		case MOTOR_PHASE_W_V_U :
			_pwm1 = pwm3;	_pwm2 = pwm2;	_pwm3 = pwm1;
			break;
	}
}

void CurrentController::ResetCurrentController(void)
{
	//	Current
	_dAxisTargetCurrent = 0.0f;
	_qAxisTargetCurrent = 0.0f;
		
	//	Controller
	_dAxisCurrentController.Reset();
	_qAxisCurrentController.Reset();
}

void CurrentController::SetDAxisTargetCurrent(float dAxisTargetCurrent)
{
	if(dAxisTargetCurrent > _maximumCurrent)	_dAxisTargetCurrent = _maximumCurrent;
	else										_dAxisTargetCurrent = dAxisTargetCurrent;
}

void CurrentController::SetQAxisTargetCurrent(float qAxisTargetCurrent)
{
	if(qAxisTargetCurrent > _maximumCurrent)	_qAxisTargetCurrent = _maximumCurrent;
	else										_qAxisTargetCurrent = qAxisTargetCurrent;
}

void CurrentController::CalculateCurrentDC(void)
{
	_dAxisActualCurrent = 0.0f;
	_qAxisActualCurrent = (_uCurrent - _vCurrent) * 0.5f;
}

void CurrentController::CalculateCurrentBLDC(void)
{
	_dAxisActualCurrent = 0.0f;
	/*
	_qAxisActualCurrent = (_uCurrent * BLDC_PHASE_U[_hallSector] + _vCurrent * BLDC_PHASE_V[_hallSector] + _wCurrent * BLDC_PHASE_W[_hallSector])*0.5f;
	
	if(_hallStatus == 1) {
		_hallSector = 0;
	}
	else if(_hallStatus == 3) {
		_hallSector = 1;
	}
	else if(_hallStatus == 2) {
		_hallSector = 2;
	}
	else if(_hallStatus == 6) {
		_hallSector = 3;
	}
	else if(_hallStatus == 4) {
		_hallSector = 4;
	}
	else if(_hallStatus == 5) {
		_hallSector = 5;
	}
	*/
}

void CurrentController::CalculateCurrentPMSM(void)
{
	ClarkeParkTransform(_elecSinTheta, _elecCosTheta, _uCurrent, _vCurrent, _wCurrent, &_dAxisActualCurrent, &_qAxisActualCurrent);
}

int8_t CurrentController::CheckOverCurrent(void)
{
	float currentLimit = _maximumCurrent * 1.2f;
	
	if(_uCurrent > HW_CURRENT_LIMIT)				_nUOverCurrent++;
	else if(_uCurrent < -HW_CURRENT_LIMIT)			_nUOverCurrent++;
	else											_nUOverCurrent--;
	if(_nUOverCurrent > NUMBER_OF_OVER_CURRENT)		return -1;
	else if(_nUOverCurrent < 0)						_nUOverCurrent = 0;
	
	if(_vCurrent > HW_CURRENT_LIMIT)				_nVOverCurrent++;
	else if(_vCurrent < -HW_CURRENT_LIMIT)			_nVOverCurrent++;
	else											_nVOverCurrent--;
	if(_nVOverCurrent > NUMBER_OF_OVER_CURRENT)		return -1;
	else if(_nVOverCurrent < 0)						_nVOverCurrent = 0;
	
	if(_wCurrent > HW_CURRENT_LIMIT)				_nWOverCurrent++;
	else if(_wCurrent < -HW_CURRENT_LIMIT)			_nWOverCurrent++;
	else											_nWOverCurrent--;
	if(_nWOverCurrent > NUMBER_OF_OVER_CURRENT)		return -1;
	else if(_nWOverCurrent < 0)						_nWOverCurrent = 0;
	
	if(_qAxisActualCurrent > currentLimit)			_nOverCurrent++;
	else if(_qAxisActualCurrent < -currentLimit)	_nOverCurrent++;
	else											_nOverCurrent--;
	if(_nOverCurrent > NUMBER_OF_OVER_CURRENT)		return -1;
	else if(_nOverCurrent < 0)						_nOverCurrent = 0;
	
	return 0;
}

void CurrentController::RunCurrentControllerDC(void)
{
	float feedforward = _backEmfVelocity * _Ke;
	
	_dAxisVoltage = 0.0f;
	_qAxisVoltage = _qAxisCurrentController.Run(_qAxisTargetCurrent + _currentOffset, _qAxisActualCurrent, feedforward, _maxOutputVoltage);
}

void CurrentController::RunCurrentControllerBLDC(void)
{
	float feedforward = _backEmfVelocity * _Ke;
	
	_dAxisVoltage = 0.0f;
	_qAxisVoltage = _qAxisCurrentController.Run(_qAxisTargetCurrent + _currentOffset, _qAxisActualCurrent, feedforward, _maxOutputVoltage);
}

void CurrentController::RunCurrentControllerPMSM(void)
{
	float feedforward = _backEmfVelocity * _Ke;
	
	_dAxisVoltage = _dAxisCurrentController.Run(_dAxisTargetCurrent, _dAxisActualCurrent, 0.0f, _maxOutputVoltage);
	_qAxisVoltage = _qAxisCurrentController.Run(_qAxisTargetCurrent + _currentOffset, _qAxisActualCurrent, feedforward, _maxOutputVoltage);
}

void CurrentController::SetElectricAngleOffset(float elecAngleOffset)
{
	_elecAngleOffset = elecAngleOffset;
}

void CurrentController::ResetElecTheta(void)
{
	_focStatus = 0;
}

void CurrentController::CalculateElecTheta(int32_t encoderPulse)
{
	if(_focStatus > 2) {
	}
	else if(_focStatus > 0) {
		if(_hallStatus != _prevHallStatus) {
			switch(_hallStatus) {
				case 6 :	if(_prevHallStatus == 2)	_elecInitAngle = -30.0f;
							else 						_elecInitAngle = 30.0f;
							break;
				case 4 :	if(_prevHallStatus == 6)	_elecInitAngle = 30.0f;
							else 						_elecInitAngle = 90.0f;
							break;
				case 5 :	if(_prevHallStatus == 4)	_elecInitAngle = 90.0f;
							else 						_elecInitAngle = 150.0f;
							break;
				case 1 :	if(_prevHallStatus == 5)	_elecInitAngle = 150.0f;
							else 						_elecInitAngle = 210.0f;
							break;
				case 3 :	if(_prevHallStatus == 1)	_elecInitAngle = 210.0f;
							else 						_elecInitAngle = 270.0f;
							break;
				case 2 :	if(_prevHallStatus == 3)	_elecInitAngle = 270.0f;
							else 						_elecInitAngle = 330.0f;
							break;
			}
			
			_focStatus++;
			
			_elecActualPosition = (int32_t)(_encoderResolution / _polePair) * (int32_t)_elecInitAngle / 360;
			_elecPrevPosition = encoderPulse;
		}
	}
	else if(_focStatus == 0) {
		switch(_hallStatus) {
			case 6 :	_elecInitAngle = 00.0f;		break;
			case 4 :	_elecInitAngle = 60.0f;		break;
			case 5 :	_elecInitAngle = 120.0f;	break;
			case 1 :	_elecInitAngle = 180.0f;	break;
			case 3 :	_elecInitAngle = 240.0f;	break;
			case 2 :	_elecInitAngle = 300.0f;	break;
			default :	_elecInitAngle = 0.0f;		break;
		}
			
		_focStatus = 1;
		
		_elecActualPosition = (int32_t)(_encoderResolution / _polePair) * (int32_t)_elecInitAngle / 360;
		_elecPrevPosition = encoderPulse;
	}
	
	_elecActualPosition += (encoderPulse - _elecPrevPosition);
	_elecPrevPosition = encoderPulse;
	
	if(_elecActualPosition > _encoderResolution) {
		_elecActualPosition -= _encoderResolution;
	}
	else if(_elecActualPosition < -_encoderResolution) {
		_elecActualPosition += _encoderResolution;
	}
	
	_elecTheta = ((float)_elecActualPosition*_pulse2deg*(float)_polePair + _elecAngleOffset) * DEG_TO_RAD;
	
	_elecSinTheta = arm_sin_f32(_elecTheta - _elecThetaError*DEG_TO_RAD);
	_elecCosTheta = arm_cos_f32(_elecTheta - _elecThetaError*DEG_TO_RAD);
	
	_prevHallStatus = _hallStatus;
}

void CurrentController::CalculateElecThetaError(void)
{

}

void CurrentController::SetCurrentPGain(float currentPGain)
{
	_dAxisCurrentController.SetPGain(currentPGain);
	_qAxisCurrentController.SetPGain(currentPGain);
}

void CurrentController::SetCurrentIGain(float currentIGain)
{
	_dAxisCurrentController.SetIGain(currentIGain);
	_qAxisCurrentController.SetIGain(currentIGain);
}
