/*
 * svpwm.cpp
 *
 *  Created on: 2021/08/03
 *      Author: yusaku
 */

#include "svpwm.h"
#include "atan.h"

// takes around 4[us] max.
void SVPWM_Calc (const AB *const ref, const float32_t vdc, UVW *const duty)
{
    //constexpr uint8_t states[7] = {0b001, 0b011, 0b010, 0b110, 0b100, 0b101, 0b001};
    //float32_t arg = atan2f(ref->B, ref->A);           // takes ~5.75us
    float32_t arg = atan2LUT(ref->B, ref->A);           // takes <1.67us
    if (arg < 0.0f)
        arg += (float32_t) (2.0f * M_PI);
    float32_t mod = sqrtf ((ref->A * ref->A) + (ref->B * ref->B)) / vdc;
    //std::cout << "arg: " << arg << ", mod: " << mod << "; ";

    int sector = arg * (float32_t) (3.0f / M_PI);
    float32_t alpha = arg - (sector * (float32_t) (M_PI / 3.0f));
    // map [0:pi/3) -> [0:128) in float
    float32_t findex = alpha * (SvpwmTableLength * (float32_t) (3.0f / M_PI));
    if (findex >= (float32_t) SvpwmTableLength)
        findex -= (float32_t) SvpwmTableLength;
    int index = findex;//((uint16_t) findex) & 0x7f;
    float32_t fract = findex - (float32_t) index;
    //std::cout << "arg: " << alpha << ", index: " << index << "; ";

    auto a = SvpwmTable[index];
    auto b = SvpwmTable[index + 1];

    float32_t ta = mod * ((1.0f - fract) * a[0] + fract * b[0]);
    float32_t tb = mod * ((1.0f - fract) * a[1] + fract * b[1]);

    //auto state_a = states[sector];
    //auto state_b = states[sector + 1];

    //duty->W = ((float32_t)((state_a & 0x100u) >> 2) * ta) + ((float32_t)((state_b & 0x100u) >> 2) * tb);
    //duty->V = ((float32_t)((state_a & 0x010u) >> 1) * ta) + ((float32_t)((state_b & 0x010u) >> 1) * tb);
    //duty->U = ((float32_t)((state_a & 0x001u) >> 0) * ta) + ((float32_t)((state_b & 0x001u) >> 0) * tb)

    // if-elif-else takes ~680ns max.
    if (sector == 0)
    {
        duty->U = ta + tb;                      // T_0/2 + T_a + T_b
        duty->V = tb;				// T_0/2 + T_b
        duty->W = 0.0f; // T_0/2
    }
    else if (sector == 1)
    {
        duty->U = ta;                           // T_0/2 + T_a
        duty->V = ta + tb;			// T_0/2 + T_a + T_b
        duty->W = 0.0f; // T_0/2
    }
    else if (sector == 2)
    {
        duty->U = 0.0f; // T_0/2
        duty->V = ta + tb;			// T_0/2 + T_a + T_b
        duty->W = tb;                           // T_0/2 + T_b
    }
    else if (sector == 3)
    {
        duty->U = 0.0f; // T_0/2
        duty->V = ta;				// T_0/2 + T_a
        duty->W = ta + tb;                      // T_0/2 + T_a + T_b
    }
    else if (sector == 4)
    {
        duty->U = tb;                           // T_0/2 + T_b
        duty->V = 0.0f;	// T_0/2
        duty->W = ta + tb;                      // T_0/2 + T_a + T_b
    }
    else if (sector == 5)
    {
        duty->U = ta + tb;                      // T_0/2 + T_a + T_b
        duty->V = 0.0f;	// T_0/2
        duty->W = ta;                           // T_0/2 + T_a
    }
    else
    {
        duty->U = 0.0f;
        duty->V = 0.0f;
        duty->W = 0.0f;
    }
}
