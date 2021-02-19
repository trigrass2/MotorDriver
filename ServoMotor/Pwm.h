#pragma once

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

void EnablePwm(void);
void DisablePwm(void);
void SetPwm(int16_t pwm1, int16_t pwm2, int16_t pwm3);

#ifdef __cplusplus
}
#endif