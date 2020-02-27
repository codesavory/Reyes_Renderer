#pragma once
#include "Types.h"

void checkboard(VertexShaderPayload& pi);
inline UVTuple get_uv(int id, int dice_factor);
void checker_explode(GeometricShaderPayload& p);
void earth(FragmentShaderPayload& p);
void blinn_phong(FragmentShaderPayload& p);
void blinn_phong_modded(FragmentShaderPayload& p);
void phong(FragmentShaderPayload& p);
