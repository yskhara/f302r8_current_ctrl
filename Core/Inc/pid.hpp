/*
 * pid.h
 *
 *  Created on: Aug 6, 2021
 *      Author: yusaku
 */

#ifndef INC_PID_HPP_
#define INC_PID_HPP_


class PiController
{
    float32_t _kp = 0.0f;
    float32_t _ki = 0.0f;
    float32_t _min = -1.0f;
    float32_t _max = 1.0f;

    PiController(float32_t kp, float32_t ti, float32_t ts, float32_t)
    {

    }
};


#endif /* INC_PID_HPP_ */
