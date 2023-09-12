#include "protect.h"
#include "CAN_receive.h"
#include "tim.h"
#include "chassis_tast.h"
#include "pm01_api.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int8_t i;
	if(htim == &htim1)
 {
     chassis_protect();
 }
 if(htim == &htim8)
 {
  chassis_move.INS_update_sign = 1;
	i++;
	if(i==20)
	{
		i = 0;
		pm01_access_poll();
	}
 }
}
