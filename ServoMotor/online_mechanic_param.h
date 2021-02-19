#pragma once

extern void ResetMcData ();
extern void AddMcData (long position, float velocity, float acceleration, float torque, float pulse2rad, float threshold);
extern bool CalcMcParam (float &J, float &B, float &C);
