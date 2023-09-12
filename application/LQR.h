#ifndef LQR_H
#define LQR_H
#include "struct_typedef.h"



typedef struct
{
    fp32 k1;
		fp32 k2;
		fp32 x1;
	  fp32 x2;
		fp32 x3;
		fp32 x4;
		fp32 out;
} LQR_type_def;

typedef struct
{   
	fp32 y[2];
	fp32 u[2];
} differ_type_def;


typedef struct
{   
	fp32 y[2];
	fp32 u[2];
	fp32 last_input;
} differ_type_def_speed;

typedef struct
{   
	fp32 y[2];
	fp32 u[2];
} lpf_type_def;

extern void LQR_init(LQR_type_def *lqr,const fp32 LQR[2]);
extern fp32 LQR_calc(LQR_type_def *lqr,fp32 INT_angle,fp32 INT_gyro,fp32 speed_input);
extern fp32 LPF(lpf_type_def *lpf ,fp32 time_cons,fp32 input,fp32 w);
extern fp32 differentiator_speed(differ_type_def_speed *differ ,fp32 bandwidth,fp32 time_cons,fp32 input);

#endif


