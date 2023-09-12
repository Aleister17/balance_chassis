#include "LQR.h"

void LQR_init(LQR_type_def *lqr,const fp32 LQR[2])
{
lqr->k1=LQR[0];
lqr->k2=LQR[1];
}

fp32 LQR_calc(LQR_type_def *lqr,fp32 INT_angle,fp32 INT_gyro,fp32 speed_input)
{
	lqr->x1=INT_angle;
	lqr->x2=speed_input;
	lqr->x3=INT_gyro;
	lqr->x4=0;
  lqr->out=-lqr->k1*(lqr->x1-lqr->x2)-lqr->k2*(lqr->x3-lqr->x4);
	return lqr->out;
}

fp32 LPF(lpf_type_def *lpf ,fp32 time_cons,fp32 input,fp32 w)
{ 
  //使用梯形法离散化
 
  lpf->u[0] = lpf->u[1];	
  lpf->u[1] = input; 
  lpf->y[0] = lpf->y[1];
  lpf->y[1] = (lpf->u[1]+lpf->u[0])/(2/time_cons + w)*w - lpf->y[0]*(w - 2/time_cons)/(2/time_cons + w);
  
  return lpf->y[1];
}


fp32 differentiator_speed(differ_type_def_speed *differ ,fp32 bandwidth,fp32 time_cons,fp32 input)
{ 
  //使用梯形法离散化

	if(fabsf(differ->last_input - input)>6.0f)//如果发生了跳变,防止产生过大的前馈量
	{
		 differ->y[0]=0;
		 differ->y[1]=0;
		 differ->u[0]=input;
		 differ->u[1]=input;
	 

	}
	
  differ->u[0] = differ->u[1];	
  differ->u[1] = input;
  differ->y[0] = differ->y[1];
  differ->y[1] = (bandwidth*(differ->u[1]-differ->u[0])-(bandwidth*time_cons/2-1)*differ->y[0])/(1+(bandwidth*time_cons)/2);
  differ->last_input = input;
	
	
	
  return differ->y[1];
	
	
}


