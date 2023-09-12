#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "LQR.h"
//#define LIMIT_JUMP_POSITION -0.5
//#define JUMP_CURRENT -20000

#define L_than_R_edc

#define		L_SLIDER_MOTOR_OFFSET		0
#define		R_SLIDER_MOTOR_OFFSET		0
#define		R_SLIDER_MOTOR_MAX	0
#define		R_SLIDER_MOTOR_MIN	0
#define		L_SLIDER_MOTOR_MAX	0
#define		L_SLIDER_MOTOR_MIN	0


//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
#define MOTOR_ECD_TO_RAD   0.000766990394f //      2*  PI  /8192



extern void chassis_init(void);
extern void chassis_task(void);
typedef double real_T;




typedef struct
{
	fp32 motor_current;//电机电流
	const motor_measure_t *motor_chassis;//电机数据指针
} chassis_motor_t;//电机控制与反馈数据
typedef struct
{
 real_T LQR_enable;
 real_T LQR_last_enable;
 real_T Upright_Ring_Output_R;
 real_T Upright_Ring_Output_L;
 real_T K1;
 real_T K2;
 LQR_type_def L_Motor;
 LQR_type_def R_Motor;
}standing_ring;




typedef struct
{
	fp32 angle1;//yaw俯仰,pitch偏航,roll翻滚
	fp32 gyro1;//x,y,z
	fp32 accel1;//x,y,z
	fp32 angle2;//yaw俯仰,pitch偏航,roll翻滚
	fp32 gyro2;//x,y,z
	fp32 accel2;//x,y,z
	fp32 angle3;//yaw俯仰,pitch偏航,roll翻滚
	fp32 gyro13;//x,y,z
	fp32 accel3;//x,y,z
} INS;





typedef struct
{   
	fp32 time_cons;//时间常数
	
	fp32 wo;//观测器带宽
	fp32 b0;//输出增益
	fp32 z1;
	fp32 z2;
	
	
	differ_type_def differ1;
	differ_type_def differ2;
    
  fp32 wc;//控制器带宽
	
  fp32 max_out;  //最大输出
	
	
	fp32 dif1;
	fp32 dif2; 
	fp32 w;//前馈带宽
	fp32 gain;//前馈增益
	
	//输入
  fp32 set;//设定值
	fp32 set_last;//上次设定值
  fp32 fdb;//反馈值
	fp32 gyro;//角速度
	fp32 err;
	
	fp32 u;	
} ladrc_fdw_type_def;




typedef struct
{

    fp32 max_relative_angle; //rad
	  fp32 max_relative_angle1;
    fp32 min_relative_angle; //rad
		fp32 min_relative_angle1;
    fp32 relative_angle;     //rad
    fp32 relative_angle1;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
		ladrc_fdw_type_def ladrc;
		uint16_t offset_ecd;
} slider_motor_t;





typedef struct
{
	fp32 vx;//x轴速度数据
	fp32 vx_set;
	fp32 wz;//z轴旋转数据
	fp32 wz_set;
	fp32 wz_out;
	chassis_motor_t motor_chassis[7];//电机数据
	chassis_motor_t motor_mf9025_chassis[2];
	
	const fp32 *INS_angle;//INS角度
	const fp32 *INS_gyro;//INS角速度
	const fp32 *INS_accel;//INS加速度
	fp32 angle[3];//yaw俯仰,pitch偏航,roll翻滚
	fp32 gyro[3];//x,y,z
	fp32 accel[3];//x,y,z
	fp32 angle_offset;//角度偏移量
	fp32 angle_offset_I;
	fp32 L_speed_feedback;//左速度反馈
	fp32 R_speed_feedback;//右速度反馈
	
	bool_t chassis_init_logo;
	float chassis_init_time;
	
	//fankuicheshi
	INS ins;

	
	//测试反馈值
	fp32 accel_g;											//重力加速度
	
	
	differ_type_def_speed L_speed;
	differ_type_def_speed R_speed;
	
	
	
	fp32 L_slider_speed_feedback;//左滑块速度反馈
	fp32 R_slider_speed_feedback;//右滑块速度反馈
	
	fp32 L_slider_motor_relative_angle;
	fp32 R_slider_motor_relative_angle;


	fp32 L_slider_motor_relative_angle_set;
	fp32 R_slider_motor_relative_angle_set;



	fp32 L_slider_speed_set;
	fp32 R_slider_speed_set;
	
	fp32 L_slider_speed;
	fp32 R_slider_speed;
	fp32 L_slider_position;
	fp32 R_slider_position;
	
	
	fp32 L_speed_set;//
	fp32 R_speed_set;
	fp32 w_gyro;
	fp32 w_wheel;
	
	pid_type_def angle_slider_control_pid;
	pid_type_def L_slider_speed_pid;
	pid_type_def R_slider_speed_pid;
	pid_type_def L_slider_position_pid;
	pid_type_def R_slider_position_pid;
	
	
	pid_type_def L_speed_pid;
	pid_type_def R_speed_pid;
	pid_type_def L_stop_pid;
	pid_type_def R_stop_pid;
	pid_type_def L_init_pid;
	pid_type_def R_init_pid;
	
	pid_type_def Angular_speed_pid;
	
	lpf_type_def L_speed_lpf;
	lpf_type_def R_speed_lpf;
	
	
	
	const Gimbal_order_t *gimbal_order;
	char jump_flag;
	char jump_over;
	standing_ring Upright_Ring_Data;
	fp32 INS_update_sign;
  char last_move_enable;
	
	fp32 L_mf9025_feedback_current;
	fp32 R_mf9025_feedback_current;
	
	fp32 L_mf9025_send_current;
	fp32 R_mf9025_send_current;
	
	fp32 L_mf9025_finish_send_current;
	fp32 R_mf9025_finish_send_current;
	
	bool_t stop_flag;
	bool_t move_flag;
	
	slider_motor_t L_slider_motor;
	slider_motor_t R_slider_motor;
	
} chassis_move_t;








extern chassis_move_t chassis_move;
extern void LADRC_FDW_init(ladrc_fdw_type_def *ladrc,fp32 wc, fp32 b0 ,fp32 wo,fp32 max_out,fp32 w,fp32 gain);
extern fp32 LADRC_FDW_calc(ladrc_fdw_type_def *ladrc, fp32 ref, fp32 set, fp32 gyro);
extern void set_cali_slider_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern fp32 differentiator(differ_type_def *differ ,fp32 bandwidth,fp32 time_cons,fp32 input);
extern fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
extern void chassis_set_control(chassis_move_t *chassis_set_control);
extern void chassis_feedback_update(chassis_move_t *chassis_feedback_update);
extern void Chassis_motor_current_limiting(chassis_move_t *Chassis_motor_current);
extern void chassis_mode_change_control_transit(chassis_move_t *chassis_mode_change);
#endif


