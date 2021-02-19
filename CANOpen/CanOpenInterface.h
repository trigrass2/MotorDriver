#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "CanOpenDef.h"

int8_t ProcessRxPDO(uint8_t *data, int8_t size);
int8_t ProcessTxPDO(uint8_t *data, int8_t size);

int8_t ProcessRxPDO1(uint16_t controlWord, int32_t targetPosition, uint32_t digitalOutput);
int8_t ProcessRxPDO2(uint16_t controlWord, int32_t targetPosition, int16_t torqueOffset, uint32_t digitalOutput);
int8_t ProcessRxPDO3(uint16_t controlWord, int32_t targetVelocity, uint32_t digitalOutput);
int8_t ProcessRxPDO4(uint16_t controlWord, int16_t targetTorque, uint32_t digitalOutput);
int8_t ProcessRxPDO5(uint16_t controlWord, int8_t modesOfOperation, int32_t targetPosition, int32_t targetVelocity, int16_t targetTorque, uint32_t digitalOutput);

int8_t ProcessTxPDO1(uint16_t *statusWord, int32_t *actualPosition, uint32_t *digitalInput);
int8_t ProcessTxPDO2(uint16_t *statusWord, int32_t *actualPosition, int32_t *actualVelocity, uint32_t *digitalInput);
int8_t ProcessTxPDO3(uint16_t *statusWord, int32_t *actualPosition, int32_t *actualVelocity, int16_t *actualTorque, uint32_t *digitalInput);
int8_t ProcessTxPDO4(uint16_t *statusWord, int32_t *actualPosition, int32_t *actualVelocity, int16_t *actualCurrent, uint32_t *digitalInput);
int8_t ProcessTxPDO5(uint16_t *statusWord, int8_t *modesOfOperationDisplay, int32_t *actualPosition, int32_t *actualVelocity, int16_t *actualTorque, uint32_t *digitalInput);

#ifdef __cplusplus
}
#endif
