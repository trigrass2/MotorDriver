#include "FieldOrientedControl.h"

#define	SQRT_3_INV_2		0.86602540378f	//	sqrt(3)/2
#define	INV_SQRT_3			0.57735026918f	//	sqrt(3)/3 //pjg<>181126 note

void ClarkeParkTransform(float sinTheta, float cosTheta, float uCurrent, float vCurrent, float wCurrent, float *dCurrent, float *qCurrent)
{
	//	Clarke Transform
	//	ids = uCurrent
	//	iqs = sqrt(3)*(uCurrent - vCurrent) / 3
	//	K = 1
	float K = 1.0f;
	float ia = K*uCurrent; //i alpha
	float ib = K*INV_SQRT_3*(uCurrent + 2.0f*vCurrent); //i beta
	

	//	Park Transform
	//	ide = ids*cosTheta + iqs*sinTheta
	//	iqe = -ids*sinTheta + iqs*cosTheta
	*dCurrent = (ia*cosTheta + ib*sinTheta)*K;
	*qCurrent = (-ia*sinTheta + ib*cosTheta)*K;
}

void InverseClarkeParkTransform(float sinTheta, float cosTheta, float dVoltage, float qVoltage, float *uVoltage, float *vVoltage, float *wVoltage)
{
	//	Inverse Park Transform
	//	vds = vde*cosTheta - vqe*sinTheta
	//	vqs = vde*sinTheta + vqe*cosTheta
	float va = dVoltage*cosTheta - qVoltage*sinTheta;
	float vb = dVoltage*sinTheta + qVoltage*cosTheta;
		
	//	Inverse Clarke Tranform
	//	uVoltage = K*vds
	//	vVoltage = K*(-0.5*vds + sqrt(3)/2*vqs)
	//	wVoltage = K*(-0.5*vds - sqrt(3)/2*vqs)
	float K = 1.0f;
	*uVoltage = K*va;
	*vVoltage = K*(-0.5f*va + SQRT_3_INV_2*vb);
	*wVoltage = K*(-0.5f*va - SQRT_3_INV_2*vb);
}

void MakeSpaceVectorPwm(float dcLinkVoltage, float maxVoltage, float uVoltage, float vVoltage, float wVoltage, int16_t maxPwm, int16_t *pwm1, int16_t *pwm2, int16_t *pwm3)
{
	float Vmax, Vmin, Vsn;
	float scaleFactor = 1.0f;
	float invDcLinkVoltage = 1.0f/dcLinkVoltage;
		
	if(uVoltage >= vVoltage) {
		Vmax = uVoltage;
		Vmin = vVoltage;
	}
	else {
		Vmax = vVoltage;
		Vmin = uVoltage;
	}
	
	if(wVoltage > Vmax) Vmax = wVoltage;
	else if(wVoltage < Vmin) Vmin = wVoltage;
	
	//	Check the overmodulation
	if((Vmax - Vmin) > (maxVoltage*2.0f)) {
		scaleFactor = (maxVoltage*2.0f) / (Vmax - Vmin);
		
		Vmax *= scaleFactor;
		Vmin *= scaleFactor;
		
		uVoltage *= scaleFactor;
		vVoltage *= scaleFactor;
		wVoltage *= scaleFactor;
	}
	
	Vsn = -(Vmax + Vmin) * 0.5f;
	uVoltage += Vsn;
	vVoltage += Vsn;
	wVoltage += Vsn;

	*pwm1 = (int16_t)((0.5f + uVoltage*invDcLinkVoltage*0.5f)*(float)maxPwm);
	*pwm2 = (int16_t)((0.5f + vVoltage*invDcLinkVoltage*0.5f)*(float)maxPwm);
	*pwm3 = (int16_t)((0.5f + wVoltage*invDcLinkVoltage*0.5f)*(float)maxPwm);
	
	if(*pwm1 < 0)	*pwm1 = 0;
	if(*pwm2 < 0)	*pwm2 = 0;
	if(*pwm3 < 0)	*pwm3 = 0;
}