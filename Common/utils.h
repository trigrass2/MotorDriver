#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <string.h>

#ifndef	NULL
#define	NULL	0
#endif

inline int8_t BytesToInt8(uint8_t *data)
{
	return (int8_t)*(data);
}

inline uint8_t BytesToUint8(uint8_t *data)
{
	return *(data);
}

inline int16_t BytesToInt16(uint8_t *data)
{
	int16_t ret;
	
	memcpy(&ret, data, 2);
	
	return ret;
}

inline uint16_t BytesToUint16(uint8_t *data)
{
	uint16_t ret;
	
	memcpy(&ret, data, 2);
	
	return ret;
}

inline int32_t BytesToInt32(uint8_t *data)
{
	int32_t ret;
	
	memcpy(&ret, data, 4);
	
	return ret;
}

inline uint32_t BytesToUint32(uint8_t *data)
{
	uint32_t ret;
	
	memcpy(&ret, data, 4);
	
	return ret;
}

inline float BytesToFloat(uint8_t *data)
{
	float ret;
	
	memcpy(&ret, data, 4);
	
	return ret;
}

inline void Int8ToBytes(int8_t value, uint8_t *data)
{
	*data = (uint8_t)value;
}

inline void Uint8ToBytes(uint8_t value, uint8_t *data)
{
	*data = value;
}

inline void Int16ToBytes(int16_t value, uint8_t *data)
{
	memcpy(data, &value, 2);
}

inline void Uint16ToBytes(uint16_t value, uint8_t *data)
{
	memcpy(data, &value, 2);
}
inline void Int32ToBytes(int32_t value, uint8_t *data)
{
	memcpy(data, &value, 4);
}

inline void Uint32ToBytes(uint32_t value, uint8_t *data)
{
	memcpy(data, &value, 4);
}

inline void FloatToBytes(float value, uint8_t *data)
{
	memcpy(data, &value, 4);
}

int32_t LoadProperties(uint8_t *buffer, uint32_t size);
int32_t SaveProperties(uint8_t *buffer, uint32_t size);

#ifdef __cplusplus
}
#endif

