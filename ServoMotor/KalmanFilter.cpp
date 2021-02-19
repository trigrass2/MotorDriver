#include <string.h>
#include "KalmanFilter.h"
#include "matrix.h"

KalmanFilter::KalmanFilter (int freq_, MotorProperty *mp_) :
	dt(1.f/freq_), mp(mp_)
{
	Init ();
}

void KalmanFilter::Init ()
{
	v_hat = 0.0f;
	Tl_hat = 0.0f;
	friction = 0.0f;

	MatrixUnit2 (P);
}

void KalmanFilter::Predict (float current)
{
	if (mp->_Jm < EPS) {
		v_hat = 0.0f;
		Tl_hat = 0.0f;
		return;
	}

	// x = (I + dt*A)*x + dt*B*u;
	// P = A*P*A' + Q
	
	float A[2][2];	// matrix A
	float At[2][2];	// transpose of A
	float B[2];		// matrix B
	float t[2][2];	// temporary matrix

	if(v_hat > 1.04719512f) {
		friction = mp->_Fv + mp->_Fc/v_hat;
	}
	else if(v_hat < -1.04719512f) {
		friction = mp->_Fv - mp->_Fc/v_hat;
	}
	else {
		friction = mp->_Fv;
	}

	At[0][0] = A[0][0] = 1.0f - dt*friction/mp->_Jm;
	At[1][0] = A[0][1] = 0.0f - dt/mp->_Jm;
	At[0][1] = A[1][0] = 0.0f;
	At[1][1] = A[1][1] = 1.0f;

	B[0] = dt*mp->_Kt/mp->_Jm;
	B[1] = 0.0f;

	// x = (I + dt*A)*x + dt*B*u
	v_hat  = A[0][0]*v_hat + A[0][1]*Tl_hat + B[0]*current;
	Tl_hat =                 A[1][1]*Tl_hat;
	
	float scale = 2.0f;
	if(v_hat > scale*mp->_maximumSpeed)							v_hat = scale*mp->_maximumSpeed;
	else if(v_hat < -scale*mp->_maximumSpeed)					v_hat = -scale*mp->_maximumSpeed;
	
	if(Tl_hat > (scale*mp->_maximumCurrent*mp->_Kt))			Tl_hat = scale*mp->_maximumCurrent*mp->_Kt;
	else if(Tl_hat < -(scale*mp->_maximumCurrent*mp->_Kt))		Tl_hat = -scale*mp->_maximumCurrent*mp->_Kt;

	// P = A*P*A' + Q
	MatrixMatrixMultiply2 (t, A, P);
	MatrixMatrixMultiply2 (P, t, At);

	float q00 = mp->_Jm;			//	For Velocity
	float q11 = mp->_Jm;			//	For Torque
	
	if(mp->_motorType == MOTOR_TYPE_LINEAR) {
		q11 *= 1000.0f;
	}
	else {
		q11 *= 0.1f;
	}

	P[0][0] += q00;
	P[1][1] += q11;
}

void KalmanFilter::Update (float velocity)
{
	// K = P*H'*!(H*P*H' + R)
	float r00 = 1.0f;
	
	float pr  = P[0][0] + r00;
	float k00 = P[0][0]/pr;
	float k10 = P[1][0]/pr;

	// x = x + K*(z - H*x)
	// float z_Hx = z - p;
	float z_Hx = velocity - v_hat;

	v_hat  += k00*z_Hx;
	Tl_hat += k10*z_Hx;

	// P = (I - K*H)*P
	float t[2][2];	// temporary matrix
	float I_KH[2][2] = {
		{1.0f - k00, 0.0f },
		{0.0f - k10, 1.0f },
	};

	MatrixMatrixMultiply2 (t, I_KH, P);

	memcpy (&P[0][0], &t[0][0], sizeof(float)*2*2);
}


