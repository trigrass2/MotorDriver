#pragma once

#define NO_POS_DATA		(1024+1)

class PositionDiff
{
private:
	int   freq;

	int   count;
	long  pdata[NO_POS_DATA];

public:
	PositionDiff (int freq_)
	{
		freq = freq_;

		count = 0;
		memset ((void *)pdata, 0, sizeof(pdata));
	}

	void Reset (long pos)
	{
		count = 0;
		for (int i=0; i<NO_POS_DATA; i++) pdata[i] = pos;
	}

	void VelocityAcceleration (long &velocity, long &acceleration, long pos, int delta)
	{
		pdata[count] = pos;

		int di = 1;
		int i0 = count;
		int i1 = count - 1*di;
		int i2 = count - 2*di;

		if (i1 < 0) i1 += NO_POS_DATA;
		if (i2 < 0) i2 += NO_POS_DATA;

		static const int dt[10] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512 };

		// 평균 속도를 계산하는 부분
		for (int i=0, n=sizeof(dt)/sizeof(int); i<n; i++) {
			int c1 = count - dt[i];
			int c2 = count - dt[i]*2;
			if (c1 < 0) c1 += NO_POS_DATA;
			if (c2 < 0) c2 += NO_POS_DATA;

			long d = (pdata[c2] - pdata[c1]) - (pdata[c1] - pdata[count]);

			if (-delta <= d && d <= delta) {
				di = dt[i];
				i1 = c1;
				i2 = c2;
			}
			else break;
		}
		if (++count == NO_POS_DATA) count = 0;

		long pd1 = pdata[i1] - pdata[i0];
		long pd2 = pdata[i2] - pdata[i0];

		velocity     = (long)((long long)(pd2 - 4*pd1)*freq/(2*di));
		acceleration = (long)((long long)(pd2 - 2*pd1)*freq*freq/(di*di));
	}
};









