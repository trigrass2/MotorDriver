#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ClarkeParkTransform(float sinTheta, float cosTheta, float uCurrent, float vCurrent, float wCurrent, float *dCurrent, float *qCurrent);
void InverseClarkeParkTransform(float sinTheta, float cosTheta, float dVoltage, float qVoltage, float *uVoltage, float *vVoltage, float *wVoltage);
void MakeSpaceVectorPwm(float dcLinkVoltage, float maxVoltage, float uVoltage, float vVoltage, float wVoltage, int16_t maxPwm, int16_t *pwm1, int16_t *pwm2, int16_t *pwm3);

#ifdef __cplusplus
}
#endif