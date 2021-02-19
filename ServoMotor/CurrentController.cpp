#include <arm_math.h>
#include "peripheral.h"
#include "CurrentController.h"
#include "FieldOrientedControl.h"
#include "stm32h743xx.h" //pjg++181105 for TIM1
#include "stdio.h" //pjg++181105 for printf

#define	BLDC_PHASE_U_TO_V	0
#define	BLDC_PHASE_U_TO_W	1
#define	BLDC_PHASE_V_TO_W	2
#define	BLDC_PHASE_V_TO_U	3
#define	BLDC_PHASE_W_TO_U	4
#define	BLDC_PHASE_W_TO_V	5

#define	HALL_ERR_CHECK1_CNT	10000 //50us*10000=>50ms
#define	ENC_ERR_GAP_CNT		1000
#define 	HALL_PATTERN_CW		1 //pjg++181015
#define 	HALL_PATTERN_CCW		-1 //pjg++181015
#define 	HALL_ENC_GAP_MIN		30
#define 	HALL_ENC_GAP_MAX		(360-HALL_ENC_GAP_MIN)

enum HALL_PATTERN_DIR { //pjg++181015
  	HPD_TYPE6M = -6,
	HPD_TYPE5M = -5,
 	HPD_TYPE4M = -4,
 	HPD_TYPE3M = -3,
 	HPD_TYPE2M = -2,
 	HPD_TYPE1M = -1,
 	HPD_TYPE0 ,
	HPD_TYPE1	= 1,
 	HPD_TYPE2,
 	HPD_TYPE3,
 	HPD_TYPE4,
 	HPD_TYPE5,
 	HPD_TYPE6,
 	HPD_TYPE_DIR_ERR,
 	HPD_TYPE_PAT_ERR,	//pattern
	HPD_TYPE_MAX
};

CurrentController::CurrentController(void)
: _dAxisCurrentController(CURRENT_CONTROLLER_PERIOD), _qAxisCurrentController(CURRENT_CONTROLLER_PERIOD)
{
	//	Voltage
	_actualVoltage = 24.0f;
	_maxOutputVoltage = _actualVoltage * 0.95f;
	_nUnderVoltageCnt = 0; //pjg++>180202
	
	//	Current
	_uCurrent = 0.0f;
	_vCurrent = 0.0f;
	_wCurrent = 0.0f;
	
	_nUOverCurrent = 0;
	_nVOverCurrent = 0;
	_nWOverCurrent = 0;
	_nOverCurrent = 0;
	_nMaxCurrentOccurCnt = 0; //pjg++180130
	_nMaxCurrent = 0;  //pjg++180130
	
	_actualCurrent = 0.0f;
	_dAxisActualCurrent = 0.0f;
	_qAxisActualCurrent = 0.0f;
	
	_dAxisPrevActualCurrent = 0.0f;
	_qAxisPrevActualCurrent = 0.0f;
	
	_dAxisAveragedCurrent = 0.0f;
	_qAxisAveragedCurrent = 0.0f;
	
	_currentOffset = 0.0f;
	_currentOffsetable = 0.0f; //pjg++181130
		
	//	Hall Sensor
	_hallStatus = 0;
	_prevHallStatus = 0;
	_hallSector = 0;
	//_hallErrCnt = 0; //pjg++180828
	
	//	Electric Angle(theta)
	_focStatus = 0;
	_elecActualPosition = 0;
	_elecPrevPosition = 0;
	//_encSameCnt = 0; //pjg++180828
	
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
	//float targetPwm = (float)MAX_PWM * _qAxisVoltage / _maxOutputVoltage;
	
	
	pwm1 = pwm2 = pwm3 = 0; //pjg++180202
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
	//_motorPhase = MOTOR_PHASE_U_W_V;
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
	
	//pjg++>18012688 //no error and down current when max current is detected
	#if 0
	if(_qAxisActualCurrent > _maximumCurrent)		_nMaxCurrent++;
	else if(_qAxisActualCurrent < -_maximumCurrent)	_nMaxCurrent++;
	else											_nMaxCurrent--;	
	if(_nMaxCurrent < 0) {
		_nMaxCurrent = 0;
		_nMaxCurrentOccurCnt = 0;
	}
	if(_nMaxCurrent > 10){//NUMBER_OF_OVER_CURRENT) {
		_nMaxCurrentOccurCnt++;
		if (_nMaxCurrentOccurCnt > 10) _qAxisTargetCurrent = _ratedCurrent;
		else {
			if (_qAxisActualCurrent > 0) {
				_qAxisTargetCurrent = _maximumCurrent - (_maximumCurrent-_ratedCurrent) *(float)(_nMaxCurrentOccurCnt/10);
			}
			else _qAxisTargetCurrent = -_maximumCurrent + (_maximumCurrent-_ratedCurrent) *(float)(_nMaxCurrentOccurCnt/10);
		}
	}
	#endif
	//pjg<++180126
	
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
	else if(_nWOverCurrent < 0)					_nWOverCurrent = 0;
	
	if(_qAxisActualCurrent > currentLimit)		{
		_nOverCurrent++;
	}
	else if(_qAxisActualCurrent < -currentLimit)	{
		_nOverCurrent++;
	}
	else	{
		_nOverCurrent--;	
	}
	if(_nOverCurrent > NUMBER_OF_OVER_CURRENT)		return -1;
	else if(_nOverCurrent < 0)	{
		_nOverCurrent = 0;
	}
    
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

//uint32_t hallerrorcnt0 = 0;
//uint32_t hallerrorcnt1 = 0;
//uint32_t hallerrorcnt2 = 0;
//uint32_t hallInputCnt = 0;
//int32_t hallInputBuf[1000];
//int32_t _hall2EncGap; //pjg++181129	
//int32_t _hallAngle; //pjg++181101
//int32_t _encAngle; //pjg++181101
#define TEST_BUF_NUM			1250
uint8_t hallBuf[TEST_BUF_NUM];
float _elecInitAngleBuf[TEST_BUF_NUM];
int32_t _elecActualPositionBuf[TEST_BUF_NUM];
uint32_t pos=0;
int32_t encoderPulseBuf[TEST_BUF_NUM];
float _elecThetaBuf[TEST_BUF_NUM];
int CurrentController::CalculateElecTheta(int32_t encoderPulse)
{
	int32_t temp, cur;
	int32_t _hall2EncGap; //pjg++181129	
	int16_t _hallAngle; //pjg++181101
	int16_t _encAngle; //pjg++181101
	long long int lltemp;//, lltemp2;
	
	if(_focStatus > 2) {
		//if (ChkHallSensorPattern()) {
		//}
	}
	else if(_focStatus > 0) {
		if(_hallStatus != _prevHallStatus) {
			switch(_hallStatus) { //CW
				case 6 :	if(_prevHallStatus == 2) {
							_elecInitAngle = -30.0f;
							_hallDir = HPD_TYPE6; //pjg<>181031
							//printf("h62");
						}
						else if(_prevHallStatus == 4) { //pjg<>181031
							_elecInitAngle = 30.0f;
							_hallDir = HPD_TYPE6M; //pjg<>181031
							//printf("h64");
						}
						else if(_prevHallStatus == 6) { //pjg<>181031
							//printf("h66");
						}
						else {
							_hallDir = HPD_TYPE_DIR_ERR; //pjg<>181031
							//printf("h6err");
							//return -2;
						}
						break;
				case 4 :	if(_prevHallStatus == 6) {
							_elecInitAngle = 30.0f;
							_hallDir = HPD_TYPE4;
							//printf("h46");
						}
						else if(_prevHallStatus == 5) {
							_elecInitAngle = 90.0f;
							_hallDir = HPD_TYPE4M;
							//printf("h45");
						}
						else if(_prevHallStatus == 4) { //pjg<>181031
							//printf("h44");
						}
						else {
							_hallDir = HPD_TYPE_DIR_ERR; //pjg<>181031
							//printf("h4err");
							//return -2;
						}
						break;
				case 5 :	if(_prevHallStatus == 4) {
							_elecInitAngle = 90.0f;
							_hallDir = HPD_TYPE5;
							//printf("h54");
						}
						else if(_prevHallStatus == 1) {
							_elecInitAngle = 150.0f;
							_hallDir = HPD_TYPE5M;
							//printf("h51");
						}
						else if(_prevHallStatus == 5) { //pjg<>181031
							//printf("h55");
						}
						else {
							_hallDir = HPD_TYPE_DIR_ERR; //pjg<>181031
							//printf("h5err");
							//return -2;
						}
						break;
				case 1 :	if(_prevHallStatus == 5) {
							_elecInitAngle = 150.0f;
							_hallDir = HPD_TYPE1;
							//printf("h15");
						}
						else if(_prevHallStatus == 3) { 
							_elecInitAngle = 210.0f;
							_hallDir = HPD_TYPE1M;
							//printf("h13");
						}
						else if(_prevHallStatus == 1) { //pjg<>181031
							//printf("h11");
						}
						else {
							_hallDir = HPD_TYPE_DIR_ERR; //pjg<>181031
							//printf("h1err");
							//return -2;
						}
						break;
				case 3 :	if(_prevHallStatus == 1) {
							_elecInitAngle = 210.0f;
							_hallDir = HPD_TYPE3;
							///printf("h31");
						}
						else if(_prevHallStatus == 2) {
							_elecInitAngle = 270.0f;
							_hallDir = HPD_TYPE2M;
							//printf("h32");
						}
						else if(_prevHallStatus == 3) { //pjg<>181031
							//printf("h33");
						}
						else {
							_hallDir = HPD_TYPE_DIR_ERR; //pjg<>181031
							//printf("h3err");
							//return -2;
						}
						break;
				case 2 :	if(_prevHallStatus == 3) {
							_elecInitAngle = 270.0f;
							_hallDir = HPD_TYPE2;
							//printf("h23");
						}
						else if(_prevHallStatus == 6) {
							_elecInitAngle = 330.0f;
							_hallDir = HPD_TYPE2M;
							//printf("h26");
						}
						else if(_prevHallStatus == 2) { //pjg<>181031
							//printf("h22");
						}
						else {
							_hallDir = HPD_TYPE_DIR_ERR; //pjg<>181031
							//printf("h2err");
							//return -2;
						}
						break;
				default : 
						_hallDir = HPD_TYPE_PAT_ERR;
						//printf("1herr");
						return -1; //sec++171204
					
			}
			
			_focStatus++;
			_elecActualPosition = (int32_t)(_encoderResolution / _polePair) * (int32_t)_elecInitAngle / 360;
			//_elecPrevPosition = encoderPulse;
		}

	}
	else if(_focStatus == 0) {
		switch(_hallStatus) {
			case 6 :
				_elecInitAngle = 00.0f;
				//printf("h6");
				break;
			case 4 :
				_elecInitAngle = 60.0f;
				//printf("h4");
				break;
			case 5 :
				_elecInitAngle = 120.0f;
				//printf("h5");
				break;
			case 1 :
				_elecInitAngle = 180.0f;
				//printf("h1");
				break;
			case 3 :
				_elecInitAngle = 240.0f;
				//printf("h3");
				break;
			case 2 :
				_elecInitAngle = 300.0f;
				//printf("h2");
				break;
			default : 
				_hallDir = HPD_TYPE_PAT_ERR;
				//printf("0herr");
				return -1;   //sec++171204
		}

		_focStatus = 1;
		_elecActualPosition = (int32_t)(_encoderResolution / _polePair) * (int32_t)_elecInitAngle / 360;
		//_elecPrevPosition = encoderPulse;
		_hallCnt = 0;
	}


	_elecActualPosition += (encoderPulse - _elecPrevPosition);
	//_elecPrevPosition = encoderPulse; //pjg--181108
	
	if (pos < TEST_BUF_NUM-1 && _qAxisTargetCurrent > 0.2) { //test
		hallBuf[pos] = _hallStatus;
		_elecInitAngleBuf[pos] = _elecInitAngle;
		_elecActualPositionBuf[pos] = _elecActualPosition;
		encoderPulseBuf[pos] = encoderPulse;
	}
	
	if(_elecActualPosition > _encoderResolution) {
		_elecActualPosition -= _encoderResolution;
	}
	else if(_elecActualPosition < -_encoderResolution) {
		_elecActualPosition += _encoderResolution;
	}
	
	_elecTheta = ((float)_elecActualPosition*_pulse2deg*(float)_polePair + _elecAngleOffset) * DEG_TO_RAD;
	if (pos < TEST_BUF_NUM-1 &&  _qAxisTargetCurrent > 0.2) { //test
		_elecThetaBuf[pos] = _elecTheta;
		pos++;
	}
	_elecSinTheta = arm_sin_f32(_elecTheta - _elecThetaError*DEG_TO_RAD);
	_elecCosTheta = arm_cos_f32(_elecTheta - _elecThetaError*DEG_TO_RAD);

	_prevHallStatus = _hallStatus;
	
	//
	//check hall/enc error
	// pjg++>181120
	//cur = TIM1->CNT;
	cur = 0;
	temp = (cur - _hallPrevPosition);
	//if (temp > 0) {
	//	hallInputBuf[hallInputCnt] = temp;
	//	hallInputCnt++;
	//	if (hallInputCnt > 999) hallInputCnt = 0;
	//}
	if(temp > 64000) {
		_hallCnt -= (65536 - temp); // 65536 : timer counter max value
	}
	else if(temp < -64000) {
		_hallCnt += (65536+temp);
	}
	else _hallCnt += temp;
	//test code
	//_hall2EncCnt = _hallCnt*166;
	//_hall2EncCnt = _hallCnt*55;//.555L;
	//_hall2EncCnt =(int32_t)( (float)_hallCnt*83.3L);
	//
	//change to degree
	lltemp = _hallCnt;
	lltemp *= 90;
	lltemp /= _polePair;
	//_hallAngle = ((_hallCnt*90)/(_polePair))%360; //note:_hallAngle = ((_hallCnt*360)/(_polePair*4))%360;
	_hallAngle = lltemp % 360;
	//_encAngle = ((_elecPrevPosition*360)/_encoderResolution)%360;
	lltemp = encoderPulse;
	lltemp *= 360;
	lltemp /= _encoderResolution;
	_encAngle = lltemp%360;
	_hall2EncGap = _hallAngle - _encAngle;
	#if 0
	if (_hall2EncGap > 0) {
		//if (_hall2EncMaxGap < _hall2EncGap && _hall2EncGap < 300) _hall2EncMaxGap = _hall2EncGap; //for check
		if (_hall2EncGap > HALL_ENC_GAP_MIN && _hall2EncGap < HALL_ENC_GAP_MAX) {
			if (_elecPrevPosition == encoderPulse) return -4;
			else if (_hallPrevPosition == cur) return -3;
			else ;//return -5;
		}
	}
	else {
		//if (_hall2EncMinGap > _hall2EncGap && _hall2EncGap > -300) _hall2EncMinGap = _hall2EncGap;
		if (_hall2EncGap < -HALL_ENC_GAP_MIN && _hall2EncGap > -HALL_ENC_GAP_MAX) {
			if (_elecPrevPosition == encoderPulse) return -4;
          		else if (_hallPrevPosition == cur) return -3;
			else ;//return -5;
		}
	}
	#endif
	if (_hallStatus == 0 || _hallStatus == 7) { //pjg++>180827
		//hallerrorcnt1++; 
		//_hallErrCnt++;
		//if (_hallErrCnt > HALL_ERR_CHECK1_CNT) 
			//return -1; //pjg++180928
	}
	//else _hallErrCnt = 0; //pjg<++180827

  	_elecPrevPosition = encoderPulse; //pjg<>181108 move pos
	_hallPrevPosition = cur;
	return 0;   //sec++171204
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

//pjg++181130
void CurrentController::SetCurrentOffset(float currentOffset)
{
	_currentOffsetable = currentOffset;
}

#if 0
//pjg++181015
int hallbufcnt = 0;
uint8_t hallbuf[100];
int CurrentController::ChkHallSensorPattern(void)
{
	int ret;

	ret = 0;
	switch(_hallStatus) { //CW
		case 6 :	
			if(_prevHallStatus == 2) {
				_hallDir = HALL_PATTERN_CW;//_elecInitAngle = -30.0f;
				_hallCnt++;
			}
			else if(_prevHallStatus == 4) {
				_hallDir = HALL_PATTERN_CCW; //_elecInitAngle = 30.0f;
				_hallCnt--;
			}
			else if(_prevHallStatus == 6) {
			}
			else {
				_hallDir = HPD_TYPE6;
			}
			break;
		case 4 :
			if(_prevHallStatus == 6) {
				_hallDir = HALL_PATTERN_CW; //	_elecInitAngle = 30.0f;
				_hallCnt++;
			}
			else if(_prevHallStatus == 5) {
				_hallDir = HALL_PATTERN_CCW; //	_elecInitAngle = 90.0f;
				_hallCnt--;
			}
			else if(_prevHallStatus == 4) {
			}
			else {
				_hallDir = HPD_TYPE4;
			}
			break;
		case 5 :
			if(_prevHallStatus == 4) {
				_hallDir = HALL_PATTERN_CW; //	_elecInitAngle = 90.0f;
				_hallCnt++;
			}
			else if(_prevHallStatus == 1) {
				_hallDir = HALL_PATTERN_CCW; //_elecInitAngle = 150.0f;
				_hallCnt--;
			}
			else if(_prevHallStatus == 5) {
			}
			else {
				_hallDir = HPD_TYPE5;
			}
			break;
		case 1 :	
			if(_prevHallStatus == 5) {
				_hallDir = HALL_PATTERN_CW; //	_elecInitAngle = 150.0f;
				_hallCnt++;
			}
			else if(_prevHallStatus == 3) {
				_hallDir = HALL_PATTERN_CCW; //	_elecInitAngle = 210.0f;
				_hallCnt--;
			}
			else if(_prevHallStatus == 1) {
			}
			else {
				_hallDir = HPD_TYPE1;
			}
			break;
		case 3 :	
			if(_prevHallStatus == 1) {
				_hallDir = HALL_PATTERN_CW; //	_elecInitAngle = 210.0f;
				_hallCnt++;
			}
			else if(_prevHallStatus == 2) {
				_hallDir = HALL_PATTERN_CCW; //_elecInitAngle = 270.0f;
				_hallCnt--;
			}
			else if(_prevHallStatus == 3) {
			}
			else {
				_hallDir = HPD_TYPE3;
			}
			break;
		case 2 :	
			if(_prevHallStatus == 3) {
				_hallDir = HALL_PATTERN_CW; //	_elecInitAngle = 270.0f;
				_hallCnt++;
			}
			else if(_prevHallStatus == 6) {
				_hallDir = HALL_PATTERN_CCW; //_elecInitAngle = 330.0f;
				_hallCnt--;
			}
			else if(_prevHallStatus == 2) {
			}
			else {
				_hallDir = HPD_TYPE2;
			}
			break;
		default : 
			_hallDir = HPD_TYPE_PAT_ERR;
	}
	//_hall2EncCnt = _hallCnt * 50;

	if (_hallStatus != _prevHallStatus) {
		hallbuf[hallbufcnt++] = _hallStatus;
		if (hallbufcnt > 99) hallbufcnt = 0;
	}
	
	if (_hallDir < HPD_TYPE_DIR_ERR) {
		if (_hallDirErrCnt) _hallDirErrCnt--;
	}
	else {
		_hallDirErrCnt++;
		if (_hallDirErrCnt > HALL_ERR_CHECK1_CNT) ret = HPD_TYPE_DIR_ERR;
	}

	return ret;
}
#endif

