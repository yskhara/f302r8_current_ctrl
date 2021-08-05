/*
 * svpwm.cpp
 *
 *  Created on: 2021/08/03
 *      Author: yusaku
 */

#include "svpwm.h"

inline constexpr float32_t saturate(const float32_t val, const float32_t min, const float32_t max) {
    if(val < min) return min;
    else if(val > max) return max;
    else return val;
}

void SVPWM_Calc(const AB * const ref, const float32_t vdc, UVW * const duty) {
	float32_t arg = atan2f(ref->B, ref->A);
	if(arg < 0.0f) arg += (float32_t)(2.0f * M_PI);
	float32_t mod = sqrtf((ref->A * ref->A) + (ref->B * ref->B)) / vdc;
	//std::cout << "arg: " << arg << ", mod: " << mod << "; ";

	    GPIOB->BSRR = GPIO_BSRR_BS_2;

	int sector = arg * (float32_t)(3.0f / M_PI);
    float32_t alpha = arg - (sector * (float32_t)(M_PI / 3.0f));
	// map [0:pi/3) -> [0:128) in float
    float32_t findex = alpha * (SvpwmTableLength * (float32_t)(3.0f / M_PI));
    if (findex >= (float32_t)SvpwmTableLength) findex -= (float32_t)SvpwmTableLength;
    int index = ((uint16_t)findex) & 0x7f;
    float32_t fract = findex - (float32_t)index;
	//std::cout << "arg: " << alpha << ", index: " << index << "; ";

	auto a = SvpwmTable[index];
	auto b = SvpwmTable[index + 1];

	float32_t ta = mod * ((1.0f-fract)*a[0] + fract*b[0]);
	float32_t tb = mod * ((1.0f-fract)*a[1] + fract*b[1]);

	UVW shadow;

	if (sector == 0) {
		shadow.U = 0.0f;	// T_0/2
		shadow.V = tb;				// T_0/2 + T_b
		shadow.W = ta + tb;			// T_0/2 + T_a + T_b
	} else if (sector == 1) {
		shadow.U = 0.0f;	// T_0/2
		shadow.V = ta + tb;			// T_0/2 + T_a + T_b
		shadow.W = ta;				// T_0/2 + T_a
	} else if (sector == 2) {
		shadow.U = tb;				// T_0/2 + T_b
		shadow.V = ta + tb;			// T_0/2 + T_a + T_b
		shadow.W = 0.0f;	// T_0/2
	} else if (sector == 3) {
		shadow.U = ta + tb;			// T_0/2 + T_a + T_b
		shadow.V = ta;				// T_0/2 + T_a
		shadow.W = 0.0f;	// T_0/2
	} else if (sector == 4) {
		shadow.U = ta + tb;			// T_0/2 + T_a + T_b
		shadow.V = 0.0f;	// T_0/2
		shadow.W = tb;				// T_0/2 + T_b
	} else if (sector == 5) {
		shadow.U = ta;				// T_0/2 + T_a
		shadow.V = 0.0f;	// T_0/2
		shadow.W = ta + tb;			// T_0/2 + T_a + T_b
	} else {
		shadow.U = 0.0f;
		shadow.V = 0.0f;
		shadow.W = 0.0f;
	}

    duty->U = saturate(shadow.U, 0.0f, 1.0f);
    duty->V = saturate(shadow.V, 0.0f, 1.0f);
    duty->W = saturate(shadow.W, 0.0f, 1.0f);
    GPIOB->BSRR = GPIO_BSRR_BR_2;
}
/*
void SVPWM_Calc(q31_t theta, q31_t modFactor, UVW *duty) {
	// Refer to Application Note AN2154 from STMicroelectronics.

	constexpr uint8_t states[7] = {0b001, 0b011, 0b010, 0b110, 0b100, 0b101, 0b001};
	constexpr q31_t fracPi3 = (0x80000000 / 3) + 1;		// PI/3 plus 1, to make sector < 6.
	constexpr q31_t frac_1_root3 = Q31(1 / 1.73205080756887729352f);

	uint32_t sector = (uint32_t)theta / (uint32_t)fracPi3;

    q31_t alpha = theta - (fracPi3 * sector);

	q31_t mul = ((q63_t)modFactor * frac_1_root3) >> 31;
    q31_t state_a_duty = (((q63_t)mul * arm_sin_q31((fracPi3 - alpha)/2)) >> 31);
    q31_t state_b_duty = (((q63_t)mul * arm_sin_q31(alpha/2)) >> 31);

    q31_t t0 = Q31(0.5f) - state_a_duty/2 - state_b_duty/2;	// T_0/2
    q31_t t1 = t0 + state_a_duty;							// T_0/2 + T_a
    q31_t t2 = t0 + state_b_duty;							// T_0/2 + T_b
    q31_t t3 = Q31(1.0f) - t0;								// T_0/2 + T_a + T_b

    switch(sector) {
    	case 0:
    	    duty->U = t0;
			duty->V = t2;
    		duty->W = t3;
    	break;
    	case 1:
    	    duty->U = t0;
			duty->V = t3;
    		duty->W = t1;
    	break;
    	case 2:
    	    duty->U = t2;
			duty->V = t3;
    		duty->W = t0;
    	break;
    	case 3:
    	    duty->U = t3;
			duty->V = t1;
    		duty->W = t0;
    	break;
    	case 4:
    	    duty->U = t3;
			duty->V = t0;
    		duty->W = t2;
    	break;
    	case 5:
    	    duty->U = t1;
			duty->V = t0;
    		duty->W = t3;
    	break;
    }
}
*/
