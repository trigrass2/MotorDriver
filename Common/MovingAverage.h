#pragma once

#include <stdint.h>

#define	MOVING_AVERAGE_MAX_BUF	50

class MovingAverage
{
public:
	MovingAverage(uint8_t dataSize, float initData);
	
protected:
	uint8_t _dataSize;
	uint8_t _index;
	float _data[MOVING_AVERAGE_MAX_BUF];
	float _dataSizeInv;
	float _dataSum;
	
public:
	float Run(float data);
};