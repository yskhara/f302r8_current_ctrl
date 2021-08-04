/*
 * coord_types.h
 *
 *  Created on: 2021/08/03
 *      Author: yusaku
 */

#ifndef INC_COORD_TYPES_H_
#define INC_COORD_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif



typedef struct {
	float32_t U;
	float32_t V;
	float32_t W;
} UVW;

typedef struct {
	float32_t D;
	float32_t Q;
} DQ;

typedef struct {
	float32_t A;
	float32_t B;
} AB;


#ifdef __cplusplus
}
#endif

#endif /* INC_COORD_TYPES_H_ */
