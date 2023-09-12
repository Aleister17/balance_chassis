#include "chassis_tast.h"
//#include "Balanced_Infantry.h"
#include "INS_task.h"
#include "tim.h"
#include "LQR.h"
#include "math.h"
#include "user_lib.h"

#define PI 3.14159265358979f
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

float current1;
float current2;
void chassis_protect(void);
const fp32 SPEED_PID[3] = {0.11,0.0003,0.01};// {0.0305,0.000000189,0}{0.0303,0.000000138,0}{0.0302,0.000000126,0}0.08  0.00075 0.075
const fp32 STOP_PID[3] = {0.02,0.001,0.1};
const fp32 INIT_PID[3] = {0.02,0.001,0.25};
const fp32 ANGULAR_SPEED_PID[3] = {2.0f,0.00,0.1};//{6,0,1.38}
const fp32 standing_ring_LQR[2] = {180.0,8.5f};//{45.7,18.7},{45.7,15.6},{45.7,14.4},{30.3936,9.0592}1.{65.3936,7.9200}{120.3936,8.9200}
const fp32 SliDER_SPEED_FEEDBACK[3] = {1.0f,0.00,0};
const fp32 ANGLE_SLIDER_CONTROL_PID[3] = {1.0f,0.00,0};
const fp32 SliDER_POSITION_FEEDBACK[3] = {1.0f,0.00,0};
fp32 L_speed_feedback2;
fp32 R_speed_feedback2;


chassis_move_t chassis_move;



void chassis_init()
{
	
	
	//���ֵ��������
	int i=0;
	int mf =0;
	
	//�������ʹ�ܱ�־
		chassis_move.Upright_Ring_Data.LQR_enable=0;
	//��ȡ��������ָ̨��͵��ָ��
		for(mf=0;mf<2;mf++)
		{
			chassis_move.motor_mf9025_chassis[mf].motor_chassis =  get_mf9025_chassis_motor_measure_point(mf);
		}
	//�����������
		chassis_move.INS_angle = get_INS_angle_point();//�Ƕ�
		chassis_move.INS_gyro = get_INS_gyro_point();//���ٶ�
		chassis_move.INS_accel = get_INS_accel_point();//���ٶ�
	//�����ָ̨��
		chassis_move.gimbal_order = get_gimbal_order_point();
	//��������������Ƿ���������������
		if(chassis_move.INS_gyro[1]==0)
		{	
			HAL_NVIC_SystemReset();
		}
	//������ֵ�趨�����鷶Χ�趨ֵ������ֵ��
		set_cali_slider_hook(L_SLIDER_MOTOR_OFFSET,R_SLIDER_MOTOR_OFFSET,L_SLIDER_MOTOR_MAX,L_SLIDER_MOTOR_MIN,R_SLIDER_MOTOR_MAX,R_SLIDER_MOTOR_OFFSET);
		PID_init(&chassis_move.angle_slider_control_pid, PID_POSITION, ANGLE_SLIDER_CONTROL_PID , 10.0f, 0.1f);
		
		
		LADRC_FDW_init(&chassis_move.L_slider_motor.ladrc,30,0.004,155,29000,50,0);
		LADRC_FDW_init(&chassis_move.R_slider_motor.ladrc,25,0.005,130,29000,40,0);

	//���̿�����
		PID_init(&chassis_move.L_speed_pid, PID_POSITION, SPEED_PID , 0.25f, 0.1f);
		PID_init(&chassis_move.R_speed_pid, PID_POSITION, SPEED_PID , 0.25f, 0.1f);
		
		PID_init(&chassis_move.L_stop_pid, PID_POSITION, STOP_PID , 0.35f, 0.1f);
		PID_init(&chassis_move.R_stop_pid, PID_POSITION, STOP_PID , 0.35f, 0.1f);
		
		PID_init(&chassis_move.L_init_pid, PID_POSITION, INIT_PID , 0.15f, 0.05f);
		PID_init(&chassis_move.R_init_pid, PID_POSITION, INIT_PID , 0.15f, 0.05f);
		
		PID_init(&chassis_move.Angular_speed_pid, PID_POSITION, ANGULAR_SPEED_PID , 10.0f, 0.1f);
	//��������
		LQR_init(&chassis_move.Upright_Ring_Data.L_Motor,standing_ring_LQR);
		LQR_init(&chassis_move.Upright_Ring_Data.R_Motor,standing_ring_LQR);
		
		HAL_Delay(100);
		__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);//������̨ʧ������
}

void chassis_task()
{
	
		//ѯ�ʵ��״̬
//		CAN_mp9025_read_motor_status1();
//		CAN_mp9025_read_motor_status2();
		//�������ݸ���
		chassis_feedback_update(&chassis_move);
	  //����ģʽ�л�����
		chassis_mode_change_control_transit(&chassis_move);
		//����ģʽ����
		chassis_set_control(&chassis_move);
		//���̵����������
		Chassis_motor_current_limiting(&chassis_move);

		CAN_mp9025_cmd_chassis_Torque_control_1(0);
//		CAN_mp9025_cmd_chassis_Torque_control_2(0);

	
	
//		CAN_mp9025_cmd_chassis_Torque_control_1((int16_t)chassis_move.motor_mf9025_chassis[0].motor_current);
//		CAN_mp9025_cmd_chassis_Torque_control_2((int16_t)chassis_move.motor_mf9025_chassis[1].motor_current);
	CAN_mp9025_read_motor_error();
	CAN_mp9025_read_motor_error1();
		//mf9025�����PID�鿴
//	CAN_mp9025_cmd_chassis_L();
//	CAN_mp9025_cmd_chassis_R();
//	CAN_mp9025_cmd_chassis((int16_t)chassis_move.motor_mf9025_chassis[0].motor_current,0);
		chassis_move.Upright_Ring_Data.LQR_last_enable =chassis_move.Upright_Ring_Data.LQR_enable;
}





void chassis_feedback_update(chassis_move_t *chassis_feedback_update)
{
		fp32 L_speed_feedback1;
		fp32 R_speed_feedback1;
		fp32 L_Encoder_value;
		fp32 R_Encoder_value;

	
//		CAN_mp9025_read_motor_status1();
//		CAN_mp9025_read_motor_status2();
	//���������ݷ�����ֵ��Ϊ�˸��ӷ��㿴����
		chassis_feedback_update->ins.angle3= chassis_feedback_update->INS_angle[2];
	//���������ݷ���
	  chassis_feedback_update->angle[0] = chassis_feedback_update->INS_angle[0];
	  chassis_feedback_update->angle[1] = chassis_feedback_update->INS_angle[1];
   	chassis_feedback_update->angle[2] = chassis_feedback_update->INS_angle[2];
	  chassis_feedback_update->gyro[0] = chassis_feedback_update->INS_gyro[0];
	  chassis_feedback_update->gyro[1] = chassis_feedback_update->INS_gyro[1];
   	chassis_feedback_update->gyro[2] = chassis_feedback_update->INS_gyro[2];
	  chassis_feedback_update->accel[0] = chassis_feedback_update->INS_accel[0];
	  chassis_feedback_update->accel[1] = chassis_feedback_update->INS_accel[1];
   	chassis_feedback_update->accel[2] = chassis_feedback_update->INS_accel[2];
	//���̽��ٶȷ�������LQR�е������ٶȷ���ֵ��������C��ı�ʱ���ı������Ǹ�ֵ
	  chassis_feedback_update->w_gyro = -chassis_feedback_update->INS_gyro[2];//���̽��ٶȣ�˳ʱ��Ϊ��
		
	//���̵���ٶȷ������˴�Ҫ���ݵ���趨ֵ���иı�
		L_speed_feedback1 =  LPF(&chassis_feedback_update->L_speed_lpf,0.002,chassis_feedback_update->motor_mf9025_chassis[0].motor_chassis->speed_rpm,25);//chassis_feedback_update->motor_mf9025_chassis[0].motor_chassis->speed_rpm;//-0.0004166666667*chassis_feedback_update->motor_mf9025_chassis[0].motor_chassis->speed_rpm;//��λm/s
		R_speed_feedback1 =  LPF(&chassis_feedback_update->R_speed_lpf,0.002,chassis_feedback_update->motor_mf9025_chassis[1].motor_chassis->speed_rpm,25);//chassis_feedback_update->motor_mf9025_chassis[1].motor_chassis->speed_rpm;//0.0004166666667*chassis_feedback_update->motor_mf9025_chassis[1].motor_chassis->speed_rpm;
		
		L_Encoder_value = (chassis_feedback_update->motor_mf9025_chassis[0].motor_chassis->ecd/65536)*3.1415;
		R_Encoder_value = (chassis_feedback_update->motor_mf9025_chassis[1].motor_chassis->ecd/65536)*3.1415;

		L_speed_feedback2 =	differentiator_speed(&chassis_feedback_update->L_speed,150,0.001,L_Encoder_value);
		R_speed_feedback2 =	differentiator_speed(&chassis_feedback_update->R_speed,150,0.001,R_Encoder_value);
		
		
		chassis_feedback_update->L_speed_feedback = chassis_feedback_update->motor_mf9025_chassis[0].motor_chassis->speed_rpm*-0.0004166666667;
		chassis_feedback_update->R_speed_feedback =	chassis_feedback_update->motor_mf9025_chassis[1].motor_chassis->speed_rpm*0.0004166666667;
		
//		chassis_feedback_update->L_speed_feedback = L_speed_feedback1*-0.0004166666667;
//		chassis_feedback_update->R_speed_feedback =	R_speed_feedback1*0.0004166666667;
	//���̵����������ֵ������������������ĵ���ֵ
		chassis_feedback_update->L_slider_speed_feedback = 0.1046f*chassis_feedback_update->motor_chassis[2].motor_chassis->speed_rpm;//��λrad/s
	  chassis_feedback_update->R_slider_speed_feedback = 0.1046f*chassis_feedback_update->motor_chassis[3].motor_chassis->speed_rpm;
		
		
		
		
		
		
		
		
	//ͨ����ֵ��������ԽǶȿ��ƣ��������Ǹ��ݻ������˵ļ�ֵȷ�����ô˴�δ��
		chassis_feedback_update->L_slider_motor_relative_angle  =		motor_ecd_to_angle_change(chassis_feedback_update->motor_chassis[2].motor_chassis->ecd,chassis_feedback_update->L_slider_motor.offset_ecd);
		chassis_feedback_update->R_slider_motor_relative_angle	=		motor_ecd_to_angle_change(chassis_feedback_update->motor_chassis[3].motor_chassis->ecd,chassis_feedback_update->R_slider_motor.offset_ecd);
	//ƽ�ⲽ��������������
		chassis_feedback_update->L_mf9025_feedback_current = chassis_feedback_update->motor_mf9025_chassis[0].motor_chassis->given_current;
		chassis_feedback_update->R_mf9025_feedback_current = chassis_feedback_update->motor_mf9025_chassis[1].motor_chassis->given_current;
		
		
	//��̨�����жϣ�����ʧ�ظ�1
	  if(chassis_move.gimbal_order->chassis_outof_control==0)
	 {
	//������̨ң������õ���Ƿ�ʹ��
		chassis_feedback_update->Upright_Ring_Data.LQR_enable = chassis_feedback_update->gimbal_order->move_enable;
	//���趨ֵ����ң������ǰ����ƫ����ֵ
		chassis_feedback_update->vx = chassis_feedback_update->gimbal_order->vx_set;
		chassis_feedback_update->wz = chassis_feedback_update->gimbal_order->wz_set;
	 }	
	 else
	 {
		chassis_feedback_update->Upright_Ring_Data.LQR_enable = 0;
		chassis_feedback_update->vx = 0;
		chassis_feedback_update->wz = 0;
		 
	 }
	//ǰ����ƫ���޷�
	  if(chassis_feedback_update->vx>35)
			 chassis_feedback_update->vx = 35;
	  if(chassis_feedback_update->vx<-35)
	     chassis_feedback_update->vx = -35;
	  if(chassis_feedback_update->wz>5)
		   chassis_feedback_update->wz = 5;
	  if(chassis_feedback_update->wz<-5)
	     chassis_feedback_update->wz = -5;
}





void chassis_set_control(chassis_move_t *chassis_set_control)
{	
	//ǰ���趨ֵ��ȡ
		chassis_set_control->vx_set =		chassis_set_control->vx/8;
	if(chassis_set_control->vx_set<0)
	{
	chassis_set_control->vx_set =	chassis_set_control->vx_set;
	}
		chassis_set_control->wz_set	=		chassis_set_control->wz;
//		//����������
//		if(chassis_set_control->Upright_Ring_Data.LQR_enable==1 && chassis_move.gimbal_order->chassis_outof_control==0)
//	 {
//	
//		 
//		 chassis_set_control->L_slider_motor_relative_angle_set = -PID_calc(&chassis_move.angle_slider_control_pid,chassis_set_control->angle[2],chassis_set_control->vx);
//		 chassis_set_control->R_slider_motor_relative_angle_set = -PID_calc(&chassis_move.angle_slider_control_pid,chassis_set_control->angle[2],chassis_set_control->vx);
//		 
//		 chassis_set_control->L_slider_position = LADRC_FDW_calc(&chassis_move.L_slider_motor.ladrc,chassis_set_control->L_slider_motor_relative_angle,chassis_set_control->L_slider_motor_relative_angle_set,chassis_set_control->L_slider_speed_feedback);
//		 chassis_set_control->R_slider_position =	LADRC_FDW_calc(&chassis_move.L_slider_motor.ladrc,chassis_set_control->L_slider_motor_relative_angle,chassis_set_control->R_slider_motor_relative_angle_set,chassis_set_control->R_slider_speed_feedback);
//		 
//		 
//		 
////			chassis_set_control->L_slider_speed = PID_calc(&chassis_set_control->L_slider_speed_pid,chassis_set_control->L_slider_speed_feedback+chassis_set_control->R_slider_speed_feedback, chassis_set_control->L_slider_position);
////			chassis_set_control->R_slider_speed = PID_calc(&chassis_set_control->R_slider_speed_pid,chassis_set_control->L_slider_speed_feedback+chassis_set_control->R_slider_speed_feedback, chassis_set_control->R_slider_position);
//			chassis_set_control->motor_chassis[2].motor_current = -chassis_set_control->L_slider_speed;//����תΪ����ֵ
//			chassis_set_control->motor_chassis[3].motor_current = -chassis_set_control->R_slider_speed;
//	 }
//		else
//	{
//				chassis_set_control->motor_chassis[2].motor_current = 0;//����תΪ����ֵ
//	      chassis_set_control->motor_chassis[3].motor_current = 0;
//	}			

	 //ƫ�������ٶȻ����㣬
	 if(chassis_set_control->Upright_Ring_Data.LQR_enable==1 && chassis_move.gimbal_order->chassis_outof_control==0)
	 {
		 
		 

	//ƫ���ǵ��������
		chassis_set_control->wz_out =-PID_calc(&chassis_set_control->Angular_speed_pid, chassis_set_control->w_gyro , chassis_set_control->wz_set);
	//Ϊ�˲��Է����Ƚ����丳ֵ��ֹ����ٶȻ�Ӱ��
//		chassis_set_control->wz_out =0;
	//�ٶȻ��������ֵ
		 if (chassis_set_control->Upright_Ring_Data.LQR_last_enable==0&&chassis_set_control->Upright_Ring_Data.LQR_enable ==1)
		 {
					chassis_set_control->chassis_init_logo = 1;

		 }
		 
			 
		
		 if(chassis_set_control->vx_set != 0)
		 {
			 chassis_set_control->stop_flag = 0;
			 if(chassis_set_control->move_flag == 0)
			 {
				 chassis_set_control->L_stop_pid.Iout = 0;
				 chassis_set_control->L_stop_pid.Iout = 0;
				 chassis_set_control->move_flag = 1;
			 }
			chassis_set_control->L_speed_pid.Iout = 0;
			chassis_set_control->R_speed_pid.Iout = 0;
			chassis_set_control->L_speed_set = PID_calc(&chassis_set_control->L_speed_pid,(chassis_set_control->L_speed_feedback+chassis_set_control->R_speed_feedback)/2, chassis_set_control->vx_set - chassis_set_control->wz_out);//- wz
			chassis_set_control->R_speed_set = PID_calc(&chassis_set_control->R_speed_pid,(chassis_set_control->L_speed_feedback+chassis_set_control->R_speed_feedback)/2, chassis_set_control->vx_set + chassis_set_control->wz_out);//+ wz
		 }
		 else
		 {
			 chassis_set_control->move_flag = 0;
			 if(chassis_set_control->stop_flag == 0)
			 {
				 chassis_set_control->L_speed_pid.Iout = 0;
				 chassis_set_control->R_speed_pid.Iout = 0;
				 chassis_set_control->stop_flag = 1;
			 }
			 chassis_set_control->L_speed_set = PID_calc(&chassis_set_control->L_stop_pid,(chassis_set_control->L_speed_feedback+chassis_set_control->R_speed_feedback)/2, chassis_set_control->vx_set - chassis_set_control->wz_out);

			 chassis_set_control->R_speed_set = PID_calc(&chassis_set_control->R_stop_pid,(chassis_set_control->L_speed_feedback+chassis_set_control->R_speed_feedback)/2, chassis_set_control->vx_set + chassis_set_control->wz_out);
																										
		 }
		if(chassis_set_control->chassis_init_logo == 1)
		{


			chassis_set_control->L_speed_set = PID_calc(&chassis_set_control->L_init_pid,(chassis_set_control->L_speed_feedback+chassis_set_control->R_speed_feedback)/2, chassis_set_control->vx_set - chassis_set_control->wz_out);//- wz
			
			chassis_set_control->R_speed_set = PID_calc(&chassis_set_control->R_init_pid,(chassis_set_control->L_speed_feedback+chassis_set_control->R_speed_feedback)/2, chassis_set_control->vx_set + chassis_set_control->wz_out);//
			
			
			
			if((fabs(chassis_set_control->L_speed_set - chassis_set_control->angle[2])<=0.1) &&(fabs(chassis_set_control->R_speed_set - chassis_set_control->angle[2])<=0.1))
			{
				chassis_set_control->chassis_init_time++;
			}
			else 
			{
				chassis_set_control->chassis_init_time =0;
			}
			
			if(chassis_set_control->chassis_init_time >1600)
			{
				chassis_set_control->chassis_init_logo = 0;
				chassis_set_control->chassis_init_time = 0;
			}
				
		}
		 
		 
	//����������ʱ��ֹ����ٶ�Ӱ�콫�丳ֵΪ0
//		chassis_set_control->L_speed_set = 0;
//		chassis_set_control->R_speed_set = 0;
	 }
	 else
	 {
		chassis_set_control->wz_out =0;
		chassis_set_control->L_speed_set = 0;
		chassis_set_control->R_speed_set = 0;
		chassis_set_control->chassis_init_time = 0;

	 }
	 
	 
	 //ֱ��������
	if(chassis_set_control->Upright_Ring_Data.LQR_enable==1&& chassis_move.gimbal_order->chassis_outof_control==0)
	{
		//ֱ�������ֵ
		chassis_set_control->Upright_Ring_Data.Upright_Ring_Output_L=LQR_calc(&chassis_move.Upright_Ring_Data.L_Motor,	chassis_set_control->angle[2],chassis_set_control->gyro[0],chassis_set_control->L_speed_set+0.002);//chassis_set_control->L_speed_set
		chassis_set_control->Upright_Ring_Data.Upright_Ring_Output_R=LQR_calc(&chassis_move.Upright_Ring_Data.R_Motor,	chassis_set_control->angle[2],chassis_set_control->gyro[0],chassis_set_control->R_speed_set+0.002);//chassis_set_control->R_speed_set
	}
	else
	{
			chassis_set_control->Upright_Ring_Data.Upright_Ring_Output_L = 0;
			chassis_set_control->Upright_Ring_Data.Upright_Ring_Output_R = 0;
	}
	if(chassis_set_control->Upright_Ring_Data.LQR_enable==1&&chassis_move.gimbal_order->chassis_outof_control==0)//��ʧ�أ��ҿ��˵���
	{
	//��������ֵ���ã���ֱ���������Ϊ�������ֵ
		chassis_set_control->motor_mf9025_chassis[0].motor_current = (0.5f*200*chassis_set_control->Upright_Ring_Data.Upright_Ring_Output_L);//����תΪ����ֵ
		chassis_set_control->motor_mf9025_chassis[1].motor_current = -(0.5f*200*chassis_set_control->Upright_Ring_Data.Upright_Ring_Output_R);
	//����������ֵ��ֵ���۲�ֵ�����㿴����
		chassis_set_control->L_mf9025_send_current = chassis_set_control->motor_mf9025_chassis[0].motor_current;
		chassis_set_control->R_mf9025_send_current = chassis_set_control->motor_mf9025_chassis[1].motor_current;
	}
	else
	{
		chassis_set_control->motor_mf9025_chassis[0].motor_current = 0;//����תΪ����ֵ
		chassis_set_control->motor_mf9025_chassis[1].motor_current = 0;
		chassis_set_control->L_mf9025_send_current = chassis_set_control->motor_mf9025_chassis[0].motor_current;
		chassis_set_control->R_mf9025_send_current = chassis_set_control->motor_mf9025_chassis[1].motor_current;
	}
		
}


void chassis_mode_change_control_transit(chassis_move_t *chassis_mode_change)
{
	//���ʹ�ܱ�־
		chassis_mode_change->last_move_enable = chassis_mode_change->gimbal_order->move_enable;
}

void LADRC_FDW_init(ladrc_fdw_type_def *ladrc,fp32 wc, fp32 b0 ,fp32 wo,fp32 max_out,fp32 w,fp32 gain)
{
    ladrc->wc = wc;
	ladrc->wo = wo;
	ladrc->b0 = b0;
	
	
	ladrc->gain = gain;
	ladrc->w = w;
	
	
	ladrc->max_out = max_out;
	ladrc->fdb = 0.0f;
	ladrc->u = 0.0f;
	ladrc->set = 0.0f;
	ladrc->gyro = 0.0f;
	ladrc->z1 = 0;
	ladrc->z2 = 0;
	ladrc->time_cons = 0.002;//������
}
fp32 differentiator(differ_type_def *differ ,fp32 bandwidth,fp32 time_cons,fp32 input)
{ 
  //ʹ�����η���ɢ��
 
  differ->u[0] = differ->u[1];	
  differ->u[1] = input;
  differ->y[0] = differ->y[1];
  differ->y[1] = (bandwidth*(differ->u[1]-differ->u[0])-(bandwidth*time_cons/2-1)*differ->y[0])/(1+(bandwidth*time_cons)/2);
  
  return differ->y[1];
}

fp32 LADRC_FDW_calc(ladrc_fdw_type_def *ladrc, fp32 ref, fp32 set, fp32 gyro)
{
	
	ladrc->err = rad_format(ladrc->set - ladrc->fdb);
	ladrc->set_last = ladrc->set;
	ladrc->set = set;
  ladrc->fdb = ref;
	ladrc->gyro = gyro;
	
	if(fabsf(ladrc->set-ladrc->set_last)>3.0f)//�������������,��ֹ���������ǰ����
	{
		 ladrc->differ1.y[0]=0;
		 ladrc->differ1.y[1]=0;
		 ladrc->differ1.u[0]=ladrc->set;
		 ladrc->differ1.u[1]=ladrc->set;
	 
		 ladrc->differ2.y[0]=0;
		 ladrc->differ2.y[1]=0;
		 ladrc->differ2.u[0]=0;
		 ladrc->differ2.u[1]=0;
	}
	
	ladrc->dif1 = differentiator(&ladrc->differ1 ,ladrc->w,ladrc->time_cons,set);
	ladrc->dif2 = differentiator(&ladrc->differ2 ,ladrc->w,ladrc->time_cons,ladrc->dif1);
	
	//��ǰ����ladrc�㷨���Ȱ�ǰ��������0�����ÿ��������ٵ���ǰ������ǰ������һ�㲻����1.
	
	//��ױ��ַ���ɢ��������
	ladrc->z2 += ladrc->time_cons*(ladrc->wo*ladrc->wo)*(ladrc->gyro-ladrc->z1);
	ladrc->z1 += ladrc->time_cons*((ladrc->b0*ladrc->u) + ladrc->z2 + (2*ladrc->wo)*(ladrc->gyro-ladrc->z1));
	ladrc->u = (ladrc->wc*ladrc->wc*ladrc->err + 2*ladrc->wc*(ladrc->gain * ladrc->dif1-ladrc->z1) - ladrc->z2 + ladrc->gain * ladrc->dif2)/ladrc->b0;
	LimitMax(ladrc->u, ladrc->max_out);
   
	return ladrc->u;
}


/**
  * @brief          ��̨У׼���ã���У׼����̨��ֵ�Լ���С����е��ԽǶ�
  * @param[in]      yaw_offse:yaw ��ֵ
  * @param[in]      pitch_offset:pitch ��ֵ
  * @param[in]      max_yaw:max_yaw:yaw �����ԽǶ�
  * @param[in]      min_yaw:yaw ��С��ԽǶ�
  * @param[in]      max_yaw:pitch �����ԽǶ�
  * @param[in]      min_yaw:pitch ��С��ԽǶ�
  * @retval         ���ؿ�
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
void set_cali_slider_hook(const uint16_t L_slider_motor_offset, const uint16_t R_slider_motor_offset, const fp32 max_L_slider_motor, const fp32 min_L_slider_motor, const fp32 max_R_slider_motor, const fp32 min_R_slider_motor)
{
    chassis_move.L_slider_motor.offset_ecd = L_slider_motor_offset;
    chassis_move.L_slider_motor.max_relative_angle = max_L_slider_motor;
    chassis_move.L_slider_motor.min_relative_angle = min_L_slider_motor;

    chassis_move.R_slider_motor.offset_ecd = R_slider_motor_offset;
    chassis_move.R_slider_motor.max_relative_angle = max_R_slider_motor;
    chassis_move.R_slider_motor.min_relative_angle = min_R_slider_motor
	;
}


/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}


//�����������
void Chassis_motor_current_limiting(chassis_move_t *Chassis_motor_current)
{
		if(chassis_move.motor_mf9025_chassis[1].motor_current>1600)
			chassis_move.motor_mf9025_chassis[1].motor_current = 1600;
		if(chassis_move.motor_mf9025_chassis[1].motor_current<-1600)
			chassis_move.motor_mf9025_chassis[1].motor_current = -1600;
		if(chassis_move.motor_mf9025_chassis[0].motor_current>1600)
			chassis_move.motor_mf9025_chassis[0].motor_current = 1600;
		if(chassis_move.motor_mf9025_chassis[0].motor_current<-1600)
			chassis_move.motor_mf9025_chassis[0].motor_current = -1600;
		
		

		
}




