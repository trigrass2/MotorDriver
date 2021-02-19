#pragma once

#include <stdint.h>

#ifndef M_PI
#define	M_PI		3.1415926536f
#endif

//	Degree to Radian
#ifndef	DEG_TO_RAD
#define	DEG_TO_RAD			0.01745329252f
#endif

//	Radian to Degree
#ifndef	RAD_TO_DEG
#define	RAD_TO_DEG			57.2957795131f
#endif

//	MPS(Meter Per Second) to MMPS(Milli Meter Per Second)
#ifndef	MPS2MMPS
#define	MPS2MMPS			1000.0f
#endif

//	MMPS(Milli Meter Per Second) to MPS(Meter Per Second)
#ifndef	MMPS2MPS
#define	MMPS2MPS			0.001f
#endif

//	RPM(Revolutions Per Minute) to RPS(Radian Per Second)
//	1rpm = 2PI/60sec
#ifndef	RPM2RPS
#define	RPM2RPS				0.1047197551f		//(	(2.0f * M_PI) / 60.0f)
#endif

//	RPS(Radian Per Second) to RPM(Revolution Per Minute)
//	1rps = 60/2PIr
#ifndef	RPS2RPM
#define	RPS2RPM				9.5492965855f		//	(60.0f / (2.0f * M_PI))
#endif

#define  EPS		(1e-8f)

inline long Round_f (float v)
{
	return (v < 0) ? (long)(v - 0.5f) : (long)(v + 0.5f);
}

inline int Sign_f(float x)
{
	if (x > 0) return 1;
	if (x < 0) return -1;
	return 0;
}

inline float Limit_f (float x, float lo, float hi)
{
	if (lo > x) return lo;
	if (hi < x) return hi;
	return x;
}

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif
