#pragma once

#include "MotorProperty.h"

class KalmanFilter {
public:
	KalmanFilter (int32_t freq_, MotorProperty *mp_);

	void Init ();
	void Predict (float current);
	void Update (float velocity);

public:
	float dt;
	MotorProperty *mp;

	float v_hat;		// velocity (rad/s or m/s)
	float Tl_hat;		// load torque	(Nm or N)
	float friction;

	// covariance matrix's elements
	float P[2][2];
};
