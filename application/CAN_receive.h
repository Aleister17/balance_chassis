/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x1ff,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

		CAN_MF9025_ALL_ID = 0x140,
		CAN_MF9025_1_ID = 0x141,
		CAN_MF9025_2_ID = 0x142,
	
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x200,
    CAN_GIMBAL_ORDER_ID = 0x712,
	  CAN_GIMBAL_MIDDLE_ID = 0x710,
	  CAN_GIMBAL_ENABLES_ID = 0x711,
} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
		int16_t num;
} motor_measure_t;
typedef struct
{
    int16_t anglePidKp;			//λ�û� P ����
    int16_t anglePidKi;			//λ�û� I ����
    int16_t speedPidKp;			//�ٶȻ� P ���� 
    int16_t speedPidKi;			//�ٶȻ� I ���� 
    int16_t iqPidKp;				//ת�ػ� P ���� 
		int16_t iqPidKi;				//ת�ػ� I ���� 
} mf3508_PI_feedback;

typedef struct
{
		int8_t temperature;
		uint16_t voltage;
		uint8_t error_flag;
}	mf9025_error_read;




typedef struct
{
	fp32 vx_set;
  fp32 wz_set;
  char move_enable;
	char suspension_enable;
	char roll_stabilize_enable;
	char jump_flag;
	
	int chassis_heat_MAX;
	char Instantaneous_power;
	int residual_joule_energy;
	
	char chassis_outof_control;
	char gimbal_update_flag;
	
} Gimbal_order_t;

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern void CAN_cmd_set_current_loop(void);

/**
  * @brief          ���ص��̵�� �������ָ��
  * @param[in]     
  * @retval         �������ָ��
  */
extern void CAN_mp9025_read_motor_error(void);
extern void CAN_mp9025_read_motor_error1(void);

extern const motor_measure_t *get_chassis_motor_measure_point(int i);
extern const motor_measure_t *get_mf9025_chassis_motor_measure_point(int i);
extern void CAN_mp9025_cmd_chassis(int16_t motor1, int16_t motor2);
extern void CAN_mp9025_cmd_chassis_Torque_control_2(int16_t motor1);
extern void CAN_mp9025_cmd_chassis_Torque_control_1(int16_t motor1);
extern void CAN_mp9025_cmd_chassis_Torque_control_R(int32_t motor1);
extern void CAN_mp9025_cmd_chassis_Torque_control_L(int32_t motor1);
extern void CAN_mp9025_cmd_chassis_L(void);
extern void CAN_mp9025_cmd_chassis_R(void);
extern void CAN_mp9025_read_motor_status1(void);
extern void CAN_mp9025_read_motor_status2(void);
extern void CAN_mp9025_cmd_chassis_PID_readin_R(void);
extern void CAN_mp9025_cmd_chassis_PID_readin_L(void);
extern const Gimbal_order_t *get_gimbal_order_point(void);
extern void chassis_protect(void);

#endif
