#include "MovingAverage.h"

MovingAverage::MovingAverage(uint8_t dataSize, float initData)
{
	if(dataSize > MOVING_AVERAGE_MAX_BUF) {
		_dataSize = MOVING_AVERAGE_MAX_BUF;
	}
	else {
		_dataSize = dataSize;
	}
	
	_index = 0;
	_dataSizeInv = 1.0f / (float)_dataSize;
	_dataSum = 0.0f;
		
	for(uint8_t i = 0; i < _dataSize; i++) {
		_data[i] = initData;
		_dataSum += initData;
	}
}

float MovingAverage::Run(float data)
{
	_dataSum -= _data[_index];
	_dataSum += data;
	_data[_index] = data;
	
	if(++_index >= _dataSize)	_index = 0;
	
	
	return (_dataSum * _dataSizeInv);
}