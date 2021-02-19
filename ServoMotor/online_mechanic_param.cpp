#include <arm_math.h>
#include "Peripheral.h"
#include "online_mechanic_param.h"

static int pos_count = 0;
static int neg_count = 0;
static int run_count = 0;

static const int MAT_SIZE = 3;
static float AtAp1[MAT_SIZE][MAT_SIZE];
static float AtBp1[MAT_SIZE];
static float AtAn1[MAT_SIZE][MAT_SIZE];
static float AtBn1[MAT_SIZE];
static float AtAp2[MAT_SIZE][MAT_SIZE];
static float AtBp2[MAT_SIZE];
static float AtAn2[MAT_SIZE][MAT_SIZE];
static float AtBn2[MAT_SIZE];
static long  p0;
static float iTe;

void ResetMcData ()
{
	run_count = 0;
	pos_count = 0;
	neg_count = 0;

	memset (AtAp1,  0, sizeof(float)*MAT_SIZE*MAT_SIZE);
	memset (AtBp1,  0, sizeof(float)*MAT_SIZE*1);
	memset (AtAn1,  0, sizeof(float)*MAT_SIZE*MAT_SIZE);
	memset (AtBn1,  0, sizeof(float)*MAT_SIZE*1);
	memset (AtAp2,  0, sizeof(float)*MAT_SIZE*MAT_SIZE);
	memset (AtBp2,  0, sizeof(float)*MAT_SIZE*1);
	memset (AtAn2,  0, sizeof(float)*MAT_SIZE*MAT_SIZE);
	memset (AtBn2,  0, sizeof(float)*MAT_SIZE*1);
}

// Te = J*a + B*w + C*sign(w) + Tl
// Te = Kt*i

// ¡òTe*dt = ¡ò(J*a + B*w + C*sign(w) + Tl)*dt + T0          C*sign(w) + Tl = Tl',	T0 = 0
// ¡òTe*dt = J*w + B*p + Tl'*t

static void AddPositiveMp1 (float v, float a, float Te)
{
	AtAp1[0][0] += a*a;	AtAp1[0][1] += a*v;	AtAp1[0][2] += a;
	AtAp1[1][0] += v*a;	AtAp1[1][1] += v*v;	AtAp1[1][2] += v;
	AtAp1[2][0] +=   a;	AtAp1[2][1] +=   v;	AtAp1[2][2] += 1;

	AtBp1[0] += a*Te;	AtBp1[1] += v*Te;	AtBp1[2] += Te;
}

static void AddNegativeMp1 (float v, float a, float Te)
{
	AtAn1[0][0] += a*a;	AtAn1[0][1] += a*v;	AtAn1[0][2] += a;
	AtAn1[1][0] += v*a;	AtAn1[1][1] += v*v;	AtAn1[1][2] += v;
	AtAn1[2][0] +=   a;	AtAn1[2][1] +=  v;	AtAn1[2][2] += 1;

	AtBn1[0] += a*Te;	AtBn1[1] += v*Te;	AtBn1[2] += Te;
}

static void AddPositiveMp2 (float t, float p, float w, float iTe)
{
	AtAp2[0][0] += w*w;	AtAp2[0][1] += w*p;	AtAp2[0][2] += w*t;
	AtAp2[1][0] += p*w;	AtAp2[1][1] += p*p;	AtAp2[1][2] += p*t;
	AtAp2[2][0] += t*w;	AtAp2[2][1] += t*p;	AtAp2[2][2] += t*t;

	AtBp2[0] += w*iTe;	AtBp2[1] += p*iTe;	AtBp2[2] += t*iTe;
}

static void AddNegativeMp2 (float t, float p, float w, float iTe)
{
	AtAn2[0][0] += w*w;	AtAn2[0][1] += w*p;	AtAn2[0][2] += w*t;
	AtAn2[1][0] += p*w;	AtAn2[1][1] += p*p;	AtAn2[1][2] += p*t;
	AtAn2[2][0] += t*w;	AtAn2[2][1] += t*p;	AtAn2[2][2] += t*t;

	AtBn2[0] += w*iTe;	AtBn2[1] += p*iTe;	AtBn2[2] += t*iTe;
}

void AddMcData (long position, float velocity, float acceleration, float torque, float pulse2rad, float threshold)
{
	if (run_count == 0) {
		iTe = 0;
		p0  = position;
	}
	iTe += torque/VELOCITY_CONTROLLER_FREQ;

	float angle = pulse2rad*(position - p0);
	float time = (float)run_count/VELOCITY_CONTROLLER_FREQ;

	if (velocity > threshold) {
		AddPositiveMp1 (velocity, acceleration, torque);
		AddPositiveMp2 (time, angle, velocity, iTe);
		pos_count++;
	}
	else if (velocity < -threshold) {
		AddNegativeMp1 (velocity, acceleration, torque);
		AddNegativeMp2 (time, angle, velocity, iTe);
		neg_count++;
	}
	else {
		run_count = 0;
		return;
	}
	run_count++;
}

bool CalcMcParam (float &J, float &B, float &C)
{
	if (pos_count < 1000 || neg_count < 1000) return false;

	int inv_success = 0;
	float AtAi[MAT_SIZE][MAT_SIZE];
	float x_data1[MAT_SIZE];
	float x_data2[MAT_SIZE];
	float x_data3[MAT_SIZE];
	float x_data4[MAT_SIZE];

	arm_matrix_instance_f32 ata1 = { MAT_SIZE, MAT_SIZE, &AtAp1[0][0]  };
	arm_matrix_instance_f32 atb1 = { MAT_SIZE, 1,        &AtBp1[0]     };
	arm_matrix_instance_f32 atai = { MAT_SIZE, MAT_SIZE, &AtAi[0][0] };

	if (ARM_MATH_SUCCESS == arm_mat_inverse_f32(&ata1, &atai)) {
		arm_matrix_instance_f32 x = { MAT_SIZE, 1, &x_data1[0] };

		arm_mat_mult_f32 (&atai, &atb1, &x);
		inv_success++;
	}

	arm_matrix_instance_f32 ata2 = { MAT_SIZE, MAT_SIZE, &AtAn1[0][0]  };
	arm_matrix_instance_f32 atb2 = { MAT_SIZE, 1,        &AtBn1[0]     };

	if (ARM_MATH_SUCCESS == arm_mat_inverse_f32(&ata2, &atai)) {
		arm_matrix_instance_f32 x = { MAT_SIZE, 1, &x_data2[0] };

		arm_mat_mult_f32 (&atai, &atb2, &x);
		inv_success++;
	}

	arm_matrix_instance_f32 ata3 = { MAT_SIZE, MAT_SIZE, &AtAp2[0][0]  };
	arm_matrix_instance_f32 atb3 = { MAT_SIZE, 1,        &AtBp2[0]     };

	if (ARM_MATH_SUCCESS == arm_mat_inverse_f32(&ata3, &atai)) {
		arm_matrix_instance_f32 x = { MAT_SIZE, 1, &x_data3[0] };

		arm_mat_mult_f32 (&atai, &atb3, &x);
		inv_success++;
	}

	arm_matrix_instance_f32 ata4 = { MAT_SIZE, MAT_SIZE, &AtAn2[0][0]  };
	arm_matrix_instance_f32 atb4 = { MAT_SIZE, 1,        &AtBn2[0]     };

	if (ARM_MATH_SUCCESS == arm_mat_inverse_f32(&ata4, &atai)) {
		arm_matrix_instance_f32 x = { MAT_SIZE, 1, &x_data4[0] };

		arm_mat_mult_f32 (&atai, &atb4, &x);
		inv_success++;
	}

	if (inv_success == 4) {
		J = (x_data3[0] + x_data4[0])/2;
		B = (x_data1[1] + x_data2[1])/2;
		C = (x_data1[2] - x_data2[2])/2;
		if (J < 0) J = 0;
		if (B < 0) B = 0;
		if (C < 0) C = 0;
		return true;
	}
	return false;
}
