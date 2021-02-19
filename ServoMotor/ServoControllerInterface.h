#pragma once

#include <stdint.h>
#include "CanOpenDef.h"
#include "MotorProperties.h"

#ifdef __cplusplus
extern "C" {
#endif

void InitServoController(void);
void LoadMotorProperty(uint32_t motorId);
int8_t SaveMotorProperty(void);


uint8_t GetEncoder1Type(void);
uint8_t GetEncoder2Type(void);

void CalculateElecAngle(uint8_t hallStatus, int32_t encoderPulse);
void SetElecTheta(float elecTheta);

void RunCurrentController(int16_t adc0, int16_t adc1);

void CalculateVelocity(int32_t encoderPulse);
void CalculateLoadTorque(void);
uint8_t RunVelocityPositionController(void);

void CalculateVoltageTemperature(uint16_t voltage, uint16_t temperature);
void SetAnalogInput(uint16_t *analogInput);


int8_t ProcessSdo(CAN_OPEN_SDO_MSG *canOpenSdoMsg);
int8_t ProcessSdoCan(uint32_t *id, uint8_t *data, uint32_t *len);
extern int Callback_MotorInfoSend(uint32_t *id, uint8_t *data, uint32_t *len); //pjg++180710

#ifdef __cplusplus
}
#endif
