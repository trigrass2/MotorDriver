#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t GetDigitalInput(void);
void SetDigitalOutput(uint32_t digitalOutput);

void TurnOnErrorLed(void);
void TurnOffErrorLed(void);

void TurnOnStatusLed(uint8_t index);
void TurnOffStatusLed(uint8_t index);

void TurnOnStatusLedAll(void);
void TurnOffStatusLedAll(void);

void TurnOnRegenerativeBrake(void);
void TurnOffRegenerativeBrake(void);

void TurnOnBrake(void);
void ReadyBrake(void);
void TurnOffBrake(void);

#ifdef __cplusplus
}
#endif