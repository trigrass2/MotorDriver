#pragma once

#include <stdint.h>
#include <assert.h>
#include <math.h>

#include "math_def.h"
#include "arm_math.h"

class TrapezoidalProfile {
private:
	float dt;
	float pulse2rad;
	float rad2pulse;

	int32_t  p_0;		// 시작 위치
	int32_t  p_e;		// 끝 위치
	float v_0;		// 초기 속도
	float v_x;		// 최고 속도(위치 프로파일), 최종 속도(속도 프로파일)

	float S, S1a, S1b, S1, S2;
	float T1, T2, T3;

	int32_t   mode;		// 프로파일의 3단계 모드(0-가속, 1-정속, 2-감속, -1-프로파일 종료)
	float t;		// 프로파일이 진행된 시간

public:
	TrapezoidalProfile (long f_) : dt(1.f/f_), mode(-1), t(0) { }

	void Reset ()
	{
		mode = -1;
	}

	void SetParam (float pulse2rad_, float rad2pulse_)
	{
		pulse2rad = pulse2rad_;
		rad2pulse = rad2pulse_;
	}

	void InitVelocityProfile (float v_tgt, float v_ini, float v_max, float a, float d)
	{
		assert (a     > EPS);
		assert (d     > EPS);
		assert (v_max > EPS);

		v_0 = v_ini;
		v_x = Limit_f (v_tgt, -v_max, v_max);

		float a0 = (fabs(v_x) > fabs(v_0)) ? a : d;
		float a1 = a0;
		
		float dv = v_x - v_0;
		T1  = max(dt, fabs(dv/a1));
		S1a = 0.5f*dv*T1;
		S1b = v_0*T1;

		S1  = S1a + S1b;
		T2  = T3 = 0;

		mode = 0;
		t = 0;
	}

	void InitPositionProfile (int32_t p_del, int32_t p_ini, float v_ini, float v_max, float a, float d)
	{
		assert (a     > EPS);
		assert (d     > EPS);
		assert (v_max > EPS);

		v_0 = v_ini;
		p_0 = p_ini;
		p_e = p_ini + p_del;
		S   = p_del*pulse2rad;

		float a1 = a;
		float a2 = d;
		
		// 현재 속도 v_0에서 가속도 a로 감속하여 정지할 경우, 이동거리 S를 벗어나는지 체크한다.
		// 만일 최고속도를 벗어나게되면, v_x 값을 최고 속도에 두고 다른 파라미터를 조절해야한다.

		// s_det를 계산하는 것은, 속도그래프가 위로 볼록한지 아래로 볼록한지를 판별하기 위해서다.
		float v_0_2 = v_0*v_0;
		float s_det = 0.5f*Sign_f(v_0)*v_0_2/a2;
		int    sign = (s_det > S) ? -1 : 1;

		// v_p2는 항상 양의 값이 되어야 한다.
		float v_p2 = (v_0_2*a2 + sign*2*a1*a2*S)/(a1 + a2);
		if (v_p2 > EPS) {
			v_x = sign*sqrt(v_p2);

			// v_peak는 v_max를 초과할 수 없기때문에 다음과 같이 v_peak를 제한한다.
			if      (v_x >  v_max) v_x =  v_max;
			else if (v_x < -v_max) v_x = -v_max;
		}
		else {
			v_x = v_0;
			if (v_0 && S) {
				a1 = a2 = 0.5f*fabs(v_0_2/S);
			}
		}

		// 가속구간(T1, S1)과 감속구간(T2, S2), 정속구간(T3, S3)에 대한 시간과 이동거리를 계산한다.
		float dv = v_x - v_0;
		T1  = max(dt, fabs(dv/a1));
		T2  = max(dt, fabs(v_x/a2));
		S1a = dv*T1/2;
		S1b = v_0*T1;
		S2  = v_x*T2/2;

		S1  = S1a + S1b;
		T3  = (v_x) ? fabs((S - S1 - S2)/v_x) : 0;
		if (T3 < dt) T3 = 0;

		mode = 0;
		t = 0;
	}

	inline float position (float S, float T, float t)	{ return     S*t*t/(T*T);	}
	inline float velocity (float S, float T, float t) 	{ return 2.f*S*t/(T*T);		}
	inline float accelera (float S, float T) 			{ return 2.f*S/(T*T);  		}

	bool VelocityProfile (float &v, float &a)
	{
		if (mode == 0) {
			if (t < T1) {
				a = accelera (S1a, T1);
				v = v_0 + velocity (S1a, T1, t);

				t += dt;
				return true;
			}
			else {
				mode = 1;
				t = 0;
				// go to next mode, do not return here.
			}
		}
		if (mode == 1) {
			a = 0;
			v = v_x;

			mode = -1;
			return true;
		}
		return false;
	}

	bool PositionProfile (int32_t &p, float &v, float &a)
	{
		if (mode == 0) {
			if (t < T1) {
				a = accelera (S1a, T1);
				v = v_0 + velocity (S1a, T1, t);
				p = p_0 + Round_f((v_0*t + position (S1a, T1, t))*rad2pulse);

				t += dt;
				return true;
			}
			else {
				mode = 1;
				t = t - T1;
				// go to next mode, do not return here.
			}
		}
		if (mode == 1) {
			if (t < T3) {
				a = 0;
				v = v_x;
				p = p_0 + Round_f((S1 + v_x*t)*rad2pulse);

				t += dt;
				return true;
			}
			else {
				mode = 2;
				t = T2 - (t - T3);
				// go to next mode, do not return here.
			}
		}
		if (mode == 2) {
			if (t > 0) {
				a = -accelera (S2, T2);
				v = velocity (S2, T2, t);
				p = p_e - Round_f(position (S2, T2, t)*rad2pulse);

				t -= dt;
				return true;
			}
			else {
				a = 0;
				v = 0;
				p = p_e;

				mode = -1;
				return true;
			}
		}
		return false;
	}
};
