#pragma once
#include "Types.h"

extern float CHECK_SIZE_X;
extern float CHECK_SIZE_Y;

void CHECKERBOARD(FragmentShaderPayload& p);
inline UVTuple get_uv(int id, int dice_factor);
void checker_explode(GeometricShaderPayload& p);
void earth(FragmentShaderPayload& p);
void blinn_phong(FragmentShaderPayload& p);
void blinn_phong_modded(FragmentShaderPayload& p);
void phong(FragmentShaderPayload& p);
